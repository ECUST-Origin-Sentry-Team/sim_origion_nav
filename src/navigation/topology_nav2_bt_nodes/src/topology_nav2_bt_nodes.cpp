#include "topology_nav2_bt_nodes/topology_nav2_bt_nodes.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nav2_behavior_tree
{

namespace
{

constexpr double GOAL_EPSILON = 1e-4;
constexpr double START_REFRESH_DISTANCE = 0.20;

}  // namespace

QueryTopologyRoute::QueryTopologyRoute(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<topology_global_planner::srv::QueryTopologyRoute>(service_node_name, conf)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList QueryTopologyRoute::providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::string>(
        "service_name",
        std::string("TopoPlanner/query_topology_route"),
        "Topology query service name"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>(
        "robot_base_frame",
        std::string("base_link_fake"),
        "Robot base frame"),
      BT::InputPort<bool>(
        "transaction_active",
        false,
        "Freeze topology query outputs while a connector transaction is active"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("wait_pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("connector_pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("exit_pose"),
      BT::OutputPort<std::string>("connector_id"),
      BT::OutputPort<std::string>("action_type"),
      BT::OutputPort<std::string>("from_region"),
      BT::OutputPort<std::string>("to_region"),
      BT::OutputPort<double>("down_timeout"),
      BT::OutputPort<double>("up_monitor_timeout"),
      BT::OutputPort<std::string>("cancel_policy"),
      BT::OutputPort<bool>("need_action")
    });
}

BT::NodeStatus QueryTopologyRoute::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_ERROR(node_->get_logger(), "QueryTopologyRoute missing input port 'goal'");
    return BT::NodeStatus::FAILURE;
  }

  bool transaction_active = false;
  getInput("transaction_active", transaction_active);

  // During wait -> down -> exit -> up, the route variables must be frozen.
  // Otherwise a Nav2 goal preemption can overwrite wait_pose / exit_pose / connector_id
  // before the connector transaction has had a chance to abort safely.
  if (transaction_active && has_cached_route_) {
    if (!isSameGoal(goal, last_goal_)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "QueryTopologyRoute detected a new goal while connector transaction is active. "
        "Keeping the cached connector route so the transaction guard can abort first.");
    }
    applyRouteOutputs(cached_route_);
    return BT::NodeStatus::SUCCESS;
  }

  if (request_sent_) {
    return check_future();
  }

  std::string global_frame = "map";
  std::string robot_base_frame = "base_link_fake";
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);

  geometry_msgs::msg::PoseStamped start;
  if (!getRobotPose(global_frame, robot_base_frame, start)) {
    return BT::NodeStatus::FAILURE;
  }

  const bool should_refresh =
    !has_cached_route_ ||
    !isSameGoal(goal, last_goal_) ||
    !has_last_start_ ||
    !isSameStart(start, last_start_);

  if (should_refresh) {
    if (!prepareRequest(start, goal)) {
      return BT::NodeStatus::FAILURE;
    }

    last_start_ = start;
    last_goal_ = goal;
    has_last_start_ = true;
    has_cached_route_ = false;
    future_result_ = service_client_->async_send_request(request_).share();
    sent_time_ = node_->now();
    request_sent_ = true;

    RCLCPP_INFO(
      node_->get_logger(),
      "QueryTopologyRoute request: start=(%.2f, %.2f), goal=(%.2f, %.2f)",
      start.pose.position.x,
      start.pose.position.y,
      goal.pose.position.x,
      goal.pose.position.y);

    return check_future();
  }

  if (!has_cached_route_) {
    return BT::NodeStatus::FAILURE;
  }

  applyRouteOutputs(cached_route_);
  return BT::NodeStatus::SUCCESS;
}

bool QueryTopologyRoute::getRobotPose(
  const std::string & global_frame,
  const std::string & robot_base_frame,
  geometry_msgs::msg::PoseStamped & pose) const
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      global_frame, robot_base_frame, tf2::TimePointZero);
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(), "QueryTopologyRoute failed to lookup %s -> %s: %s",
      global_frame.c_str(), robot_base_frame.c_str(), ex.what());
    return false;
  }
}

void QueryTopologyRoute::on_tick()
{
  // This node overrides tick() to support route caching and transaction freezing.
  // The request is filled in tick() immediately before async_send_request().
}

bool QueryTopologyRoute::prepareRequest(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  request_->start = start;
  request_->goal = goal;
  return true;
}

BT::NodeStatus QueryTopologyRoute::on_completion(
  std::shared_ptr<topology_global_planner::srv::QueryTopologyRoute::Response> response)
{
  RCLCPP_WARN(
    node_->get_logger(),
    "QueryTopologyRoute response: success=%d need_action=%d message='%s' "
    "connector_id='%s' action_type='%s' from='%s' to='%s' "
    "final=(%.2f, %.2f) wait=(%.2f, %.2f) exit=(%.2f, %.2f)",
    response->success,
    response->need_action,
    response->message.c_str(),
    response->connector_id.c_str(),
    response->action_type.c_str(),
    response->from_region.c_str(),
    response->to_region.c_str(),
    response->final_goal.pose.position.x,
    response->final_goal.pose.position.y,
    response->wait_pose.pose.position.x,
    response->wait_pose.pose.position.y,
    response->exit_pose.pose.position.x,
    response->exit_pose.pose.position.y);

  if (!response->success) {
    RCLCPP_WARN(node_->get_logger(), "QueryTopologyRoute failed: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  cached_route_ = *response;
  has_cached_route_ = true;
  applyRouteOutputs(cached_route_);
  return BT::NodeStatus::SUCCESS;
}

void QueryTopologyRoute::applyRouteOutputs(const QueryTopologyRouteSrv::Response & response)
{
  setOutput("final_goal", response.final_goal);
  setOutput("wait_pose", response.wait_pose);
  setOutput("connector_pose", response.connector_pose);
  setOutput("exit_pose", response.exit_pose);
  setOutput("connector_id", response.connector_id);
  setOutput("action_type", response.action_type);
  setOutput("from_region", response.from_region);
  setOutput("to_region", response.to_region);
  setOutput("down_timeout", response.down_timeout);
  setOutput("up_monitor_timeout", response.up_monitor_timeout);
  setOutput("cancel_policy", response.cancel_policy);
  setOutput("need_action", response.need_action);
}

bool QueryTopologyRoute::isSameGoal(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs) const
{
  return lhs.header.frame_id == rhs.header.frame_id &&
         std::fabs(lhs.pose.position.x - rhs.pose.position.x) < GOAL_EPSILON &&
         std::fabs(lhs.pose.position.y - rhs.pose.position.y) < GOAL_EPSILON &&
         std::fabs(lhs.pose.position.z - rhs.pose.position.z) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.x - rhs.pose.orientation.x) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.y - rhs.pose.orientation.y) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.z - rhs.pose.orientation.z) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.w - rhs.pose.orientation.w) < GOAL_EPSILON;
}

bool QueryTopologyRoute::isSameStart(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs) const
{
  const double dx = lhs.pose.position.x - rhs.pose.position.x;
  const double dy = lhs.pose.position.y - rhs.pose.position.y;
  return std::hypot(dx, dy) < START_REFRESH_DISTANCE;
}

// ------------------------------------------------------------------------ //

IsConnectorActionRequired::IsConnectorActionRequired(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}

BT::PortsList IsConnectorActionRequired::providedPorts()
{
  return {
    BT::InputPort<bool>("need_action")
  };
}

BT::NodeStatus IsConnectorActionRequired::tick()
{
  bool need_action = false;
  if (!getInput("need_action", need_action)) {
    throw BT::RuntimeError("IsConnectorActionRequired missing input [need_action]");
  }

  RCLCPP_INFO(
    rclcpp::get_logger("IsConnectorActionRequired"),
    "IsConnectorActionRequired tick: need_action=%d",
    need_action);

  return need_action ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------------------------------------------------------------ //

SetConnectorStage::SetConnectorStage(
  const std::string & node_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(node_name, conf)
{
}

BT::PortsList SetConnectorStage::providedPorts()
{
  return {
    BT::InputPort<std::string>("stage"),
    BT::InputPort<bool>(
      "yield_once",
      false,
      "Return RUNNING for one tick after setting the stage so the transaction guard can react"),
    BT::OutputPort<std::string>("connector_stage")
  };
}

BT::NodeStatus SetConnectorStage::onStart()
{
  std::string stage;
  if (!getInput("stage", stage)) {
    throw BT::RuntimeError("SetConnectorStage missing input [stage]");
  }

  setOutput("connector_stage", stage);
  yielded_once_ = false;

  bool yield_once = false;
  getInput("yield_once", yield_once);

  RCLCPP_INFO(
    rclcpp::get_logger("SetConnectorStage"),
    "Connector stage set to '%s' yield_once=%d",
    stage.c_str(),
    yield_once);

  if (yield_once) {
    yielded_once_ = true;
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetConnectorStage::onRunning()
{
  if (yielded_once_) {
    yielded_once_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::SUCCESS;
}

void SetConnectorStage::onHalted()
{
  yielded_once_ = false;
}

// ------------------------------------------------------------------------ //

SetHeadCommand::SetHeadCommand(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<topology_global_planner::srv::ExecuteConnectorAction>(service_node_name, conf)
{
}

BT::PortsList SetHeadCommand::providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<std::string>(
        "service_name",
        std::string("/connector_action_manager/execute"),
        "Connector action manager service"),
      BT::InputPort<bool>("down", false, "Whether the head-down flag should be true")
    });
}

void SetHeadCommand::on_tick()
{
  bool down = false;
  getInput("down", down);

  request_->session_id = "__head_command__";
  request_->connector_id = "__head_command__";
  request_->action_type = "down";
  request_->phase = down ? "force_down" : "force_up";
  request_->timeout = 0.0;
  request_->cancel_policy = down ? "down_if_safe" : "up_if_safe";

  RCLCPP_INFO(
    node_->get_logger(),
    "SetHeadCommand request: down=%d phase='%s'",
    down,
    request_->phase.c_str());
}

BT::NodeStatus SetHeadCommand::on_completion(
  std::shared_ptr<topology_global_planner::srv::ExecuteConnectorAction::Response> response)
{
  if (!response->success) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SetHeadCommand failed: %s",
      response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

// ------------------------------------------------------------------------ //

ConnectorTransactionGuard::ConnectorTransactionGuard(
  const std::string & node_name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(node_name, conf)
{
}

BT::PortsList ConnectorTransactionGuard::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "service_name",
      std::string("/connector_action_manager/execute"),
      "Connector action manager service"),
    BT::InputPort<std::string>("session_id", std::string(""), "Connector traversal session id"),
    BT::InputPort<std::string>("action_type"),
    BT::InputPort<std::string>("connector_id"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("connector_pose"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("wait_pose"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("exit_pose"),
    BT::InputPort<std::string>(
      "cancel_policy",
      std::string("up_if_safe"),
      "Connector cancel policy used for ordinary halt/failure"),
    BT::InputPort<std::string>(
      "preempt_cancel_policy",
      std::string("down_if_safe"),
      "Cancel policy used when a new NavigateToPose goal preempts an active connector transaction"),
    BT::InputPort<std::string>(
      "connector_stage",
      std::string("IDLE"),
      "Current connector transaction stage"),
    BT::OutputPort<bool>(
      "transaction_active",
      "True while the connector transaction child is running")
  };
}

BT::NodeStatus ConnectorTransactionGuard::tick()
{
  if (!ensureNode()) {
    return BT::NodeStatus::FAILURE;
  }

  current_stage_ = getCurrentStage();

  if (!transaction_active_) {
    if (!captureInputs()) {
      return BT::NodeStatus::FAILURE;
    }
    transaction_active_ = true;
    abort_sent_ = false;
    preempt_pending_ = false;
    current_stage_ = getCurrentStage();
    setTransactionActive(true);
    RCLCPP_INFO(
      node_->get_logger(),
      "ConnectorTransactionGuard started: connector='%s', session='%s', stage='%s'",
      abort_request_.connector_id.c_str(),
      abort_request_.session_id.c_str(),
      current_stage_.c_str());
  }

  if (transaction_active_ && isGoalPreempted()) {
    if (shouldDeferPreempt(current_stage_)) {
      preempt_pending_ = true;
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "ConnectorTransactionGuard defers goal preemption during critical stage '%s'. "
        "The robot will keep the head down and first reach the connector exit pose.",
        current_stage_.c_str());
    } else {
      const std::string policy = preemptPolicyForStage(current_stage_);
      RCLCPP_WARN(
        node_->get_logger(),
        "ConnectorTransactionGuard handles goal preemption at stage '%s' with policy '%s'.",
        current_stage_.c_str(),
        policy.c_str());
      sendAbort("goal_preempted", policy);
      haltChild();
      transaction_active_ = false;
      preempt_pending_ = false;
      setTransactionActive(false);

      // Do not return FAILURE here. A FAILURE would bubble to the outer RecoveryNode
      // and can trigger ClearCostmap/Escape before the new goal is re-queried.
      return BT::NodeStatus::RUNNING;
    }
  }

  if (preempt_pending_ &&
      (current_stage_ == "AT_EXIT_BEFORE_UP" || current_stage_ == "WAITING_FOR_UP"))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "ConnectorTransactionGuard reached safe exit stage '%s' with a pending preempt. "
      "Holding/requesting head down and replanning before any head-up command.",
      current_stage_.c_str());
    sendAbort("deferred_goal_preempted", "down_if_safe");
    haltChild();
    transaction_active_ = false;
    preempt_pending_ = false;
    setTransactionActive(false);
    return BT::NodeStatus::RUNNING;
  }

  const BT::NodeStatus child_status = child_node_->executeTick();

  if (child_status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }

  if (child_status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ConnectorTransactionGuard finished successfully: connector='%s'",
      abort_request_.connector_id.c_str());
    transaction_active_ = false;
    setTransactionActive(false);
    return BT::NodeStatus::SUCCESS;
  }

  sendAbort("child_failure", failurePolicyForStage(current_stage_));
  transaction_active_ = false;
  preempt_pending_ = false;
  setTransactionActive(false);
  return BT::NodeStatus::FAILURE;
}

void ConnectorTransactionGuard::halt()
{
  if (transaction_active_) {
    sendAbort("halt", failurePolicyForStage(current_stage_));
  }
  transaction_active_ = false;
  preempt_pending_ = false;
  setTransactionActive(false);
  BT::DecoratorNode::halt();
}

bool ConnectorTransactionGuard::ensureNode()
{
  if (node_) {
    return true;
  }

  if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ConnectorTransactionGuard"),
      "ConnectorTransactionGuard failed to get rclcpp node from blackboard");
    return false;
  }

  return true;
}

bool ConnectorTransactionGuard::captureInputs()
{
  abort_request_ = ExecuteConnectorActionSrv::Request();
  abort_request_.phase = "abort";
  abort_request_.timeout = 0.0;
  abort_request_.cancel_policy = "up_if_safe";

  getInput("session_id", abort_request_.session_id);
  getInput("cancel_policy", abort_request_.cancel_policy);

  if (!getInput("action_type", abort_request_.action_type)) {
    RCLCPP_ERROR(node_->get_logger(), "ConnectorTransactionGuard missing input port 'action_type'");
    return false;
  }

  if (!getInput("connector_id", abort_request_.connector_id)) {
    RCLCPP_ERROR(node_->get_logger(), "ConnectorTransactionGuard missing input port 'connector_id'");
    return false;
  }

  if (!getInput("connector_pose", abort_request_.connector_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "ConnectorTransactionGuard missing input port 'connector_pose'");
    return false;
  }

  if (!getInput("wait_pose", abort_request_.wait_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "ConnectorTransactionGuard missing input port 'wait_pose'");
    return false;
  }

  if (!getInput("exit_pose", abort_request_.exit_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "ConnectorTransactionGuard missing input port 'exit_pose'");
    return false;
  }

  if (abort_request_.session_id.empty()) {
    abort_request_.session_id = abort_request_.connector_id;
  }

  geometry_msgs::msg::PoseStamped goal;
  if (getInput("goal", goal)) {
    active_goal_ = goal;
    has_active_goal_ = true;
  } else {
    has_active_goal_ = false;
  }

  return true;
}

std::string ConnectorTransactionGuard::getCurrentStage()
{
  std::string stage = "IDLE";
  getInput("connector_stage", stage);
  return stage;
}

bool ConnectorTransactionGuard::isGoalPreempted()
{
  geometry_msgs::msg::PoseStamped current_goal;
  const bool has_current_goal = getInput("goal", current_goal).has_value();
  return has_active_goal_ && has_current_goal && !isSameGoal(active_goal_, current_goal);
}

bool ConnectorTransactionGuard::shouldDeferPreempt(const std::string & stage) const
{
  // Once the head is confirmed down and the robot is committed to the connector,
  // a new goal must not interrupt the crossing. Finish the old exit pose first,
  // then re-query the new goal before any head-up command is sent.
  return stage == "DOWN_DONE" || stage == "CROSSING_TO_EXIT";
}

std::string ConnectorTransactionGuard::preemptPolicyForStage(const std::string & stage)
{
  // Not yet in the connector: stop the down action and allow ordinary replanning.
  if (stage == "APPROACHING_WAIT" ||
      stage == "AT_WAIT_BEFORE_DOWN" ||
      stage == "LOWERING" ||
      stage == "IDLE")
  {
    return "up_if_safe";
  }

  // At or after the connector exit, but before the new route is known: keep the
  // head down. If the new route does not need a connector, NormalNavigate will
  // explicitly send force_up before moving.
  if (stage == "AT_EXIT_BEFORE_UP" || stage == "WAITING_FOR_UP") {
    return "down_if_safe";
  }

  // After UP_DONE it is safe to keep/request up.
  if (stage == "UP_DONE") {
    return "up_if_safe";
  }

  std::string xml_policy = "down_if_safe";
  getInput("preempt_cancel_policy", xml_policy);
  return xml_policy;
}

std::string ConnectorTransactionGuard::failurePolicyForStage(const std::string & stage) const
{
  if (stage == "DOWN_DONE" ||
      stage == "CROSSING_TO_EXIT" ||
      stage == "AT_EXIT_BEFORE_UP" ||
      stage == "WAITING_FOR_UP")
  {
    return "down_if_safe";
  }

  return "up_if_safe";
}

void ConnectorTransactionGuard::setTransactionActive(bool active)
{
  setOutput("transaction_active", active);
}

void ConnectorTransactionGuard::sendAbort(
  const std::string & reason,
  const std::string & policy_override)
{
  if (!ensureNode() || abort_sent_) {
    return;
  }

  std::string service_name = "/connector_action_manager/execute";
  getInput("service_name", service_name);

  if (!abort_client_) {
    abort_client_ = node_->create_client<ExecuteConnectorActionSrv>(service_name);
  }

  if (!abort_client_->wait_for_service(200ms)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ConnectorTransactionGuard could not reach abort service '%s' during %s",
      service_name.c_str(),
      reason.c_str());
    return;
  }

  auto request = std::make_shared<ExecuteConnectorActionSrv::Request>(abort_request_);
  request->phase = "abort";
  request->timeout = 0.0;
  if (!policy_override.empty()) {
    request->cancel_policy = policy_override;
  }
  abort_client_->async_send_request(request);
  abort_sent_ = true;

  RCLCPP_WARN(
    node_->get_logger(),
    "ConnectorTransactionGuard sent abort: reason='%s', connector='%s', session='%s', policy='%s'",
    reason.c_str(),
    request->connector_id.c_str(),
    request->session_id.c_str(),
    request->cancel_policy.c_str());
}

bool ConnectorTransactionGuard::isSameGoal(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs) const
{
  return lhs.header.frame_id == rhs.header.frame_id &&
         std::fabs(lhs.pose.position.x - rhs.pose.position.x) < GOAL_EPSILON &&
         std::fabs(lhs.pose.position.y - rhs.pose.position.y) < GOAL_EPSILON &&
         std::fabs(lhs.pose.position.z - rhs.pose.position.z) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.x - rhs.pose.orientation.x) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.y - rhs.pose.orientation.y) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.z - rhs.pose.orientation.z) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.w - rhs.pose.orientation.w) < GOAL_EPSILON;
}

// ------------------------------------------------------------------------ //

ExecuteConnectorAction::ExecuteConnectorAction(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<topology_global_planner::srv::ExecuteConnectorAction>(service_node_name, conf)
{
}

BT::PortsList ExecuteConnectorAction::providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<std::string>(
        "service_name",
        std::string("/connector_action_manager/execute"),
        "Connector action manager service"),
      BT::InputPort<std::string>("session_id", std::string(""), "Connector traversal session id"),
      BT::InputPort<std::string>("phase", std::string("pre"), "Connector traversal phase"),
      BT::InputPort<std::string>("action_type"),
      BT::InputPort<std::string>("connector_id"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("connector_pose"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("wait_pose"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("exit_pose"),
      BT::InputPort<double>("timeout", 0.0, "Connector action phase timeout"),
      BT::InputPort<std::string>(
        "cancel_policy",
        std::string("up_if_safe"),
        "Connector cancel policy")
    });
}

void ExecuteConnectorAction::on_tick()
{
  request_->session_id.clear();
  request_->phase = "pre";
  request_->timeout = 0.0;
  request_->cancel_policy = "up_if_safe";
  should_send_request_ = true;

  geometry_msgs::msg::PoseStamped goal;
  if (getInput("goal", goal)) {
    active_goal_ = goal;
    has_active_goal_ = true;
  }

  getInput("session_id", request_->session_id);
  getInput("phase", request_->phase);

  if (!getInput("action_type", request_->action_type)) {
    RCLCPP_ERROR(node_->get_logger(), "ExecuteConnectorAction missing input port 'action_type'");
    should_send_request_ = false;
    return;
  }

  if (!getInput("connector_id", request_->connector_id)) {
    RCLCPP_ERROR(node_->get_logger(), "ExecuteConnectorAction missing input port 'connector_id'");
    should_send_request_ = false;
    return;
  }

  if (!getInput("connector_pose", request_->connector_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "ExecuteConnectorAction missing input port 'connector_pose'");
    should_send_request_ = false;
    return;
  }

  if (!getInput("wait_pose", request_->wait_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "ExecuteConnectorAction missing input port 'wait_pose'");
    should_send_request_ = false;
    return;
  }

  if (!getInput("exit_pose", request_->exit_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "ExecuteConnectorAction missing input port 'exit_pose'");
    should_send_request_ = false;
    return;
  }

  if (request_->session_id.empty()) {
    request_->session_id = request_->connector_id;
  }

  getInput("timeout", request_->timeout);
  getInput("cancel_policy", request_->cancel_policy);

  RCLCPP_INFO(
    node_->get_logger(),
    "ExecuteConnectorAction request: phase='%s', connector='%s', session='%s', timeout=%.2f, policy='%s'",
    request_->phase.c_str(),
    request_->connector_id.c_str(),
    request_->session_id.c_str(),
    request_->timeout,
    request_->cancel_policy.c_str());
}

BT::NodeStatus ExecuteConnectorAction::on_completion(
  std::shared_ptr<topology_global_planner::srv::ExecuteConnectorAction::Response> response)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "ExecuteConnectorAction completion: success=%d, message='%s', new_state='%s'",
    response->success,
    response->message.c_str(),
    response->new_state.c_str());

  if (!response->success) {
    RCLCPP_WARN(
      node_->get_logger(), "ExecuteConnectorAction failed: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void ExecuteConnectorAction::halt()
{
  // The connector transaction as a whole is owned by ConnectorTransactionGuard.
  // Do not send an abort from this individual service node: if a goal preemption
  // happens while ExecuteConnectorAction is RUNNING, the guard has already sent
  // the abort with the correct preempt_cancel_policy. Sending a second abort here
  // can overwrite that policy, e.g. down_if_safe followed by an unintended up_if_safe.
  RCLCPP_INFO(
    node_->get_logger(),
    "ExecuteConnectorAction halted during phase '%s' for connector '%s' without local abort",
    request_->phase.c_str(),
    request_->connector_id.c_str());
  BtServiceNode<topology_global_planner::srv::ExecuteConnectorAction>::halt();
}

bool ExecuteConnectorAction::isSameGoal(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs) const
{
  return lhs.header.frame_id == rhs.header.frame_id &&
         std::fabs(lhs.pose.position.x - rhs.pose.position.x) < GOAL_EPSILON &&
         std::fabs(lhs.pose.position.y - rhs.pose.position.y) < GOAL_EPSILON &&
         std::fabs(lhs.pose.position.z - rhs.pose.position.z) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.x - rhs.pose.orientation.x) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.y - rhs.pose.orientation.y) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.z - rhs.pose.orientation.z) < GOAL_EPSILON &&
         std::fabs(lhs.pose.orientation.w - rhs.pose.orientation.w) < GOAL_EPSILON;
}

// ------------------------------------------------------------------------ //

IsHeadState::IsHeadState(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(action_name, conf)
{
}

BT::PortsList IsHeadState::providedPorts()
{
  return {
    BT::InputPort<std::string>("expected_state", "DOWN_DONE", "Expected head state")
  };
}

void IsHeadState::headStateCallback(const referee_msg::msg::Referee::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_head_state_ = static_cast<int8_t>(msg->head_status);
  has_head_state_ = true;
  RCLCPP_INFO(
    node_->get_logger(),
    "IsHeadState callback: head_status=%d, last_head_state_=%d",
    msg->head_status, last_head_state_);
}

BT::NodeStatus IsHeadState::onStart()
{
  if (!initialized_) {
    config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    head_state_sub_ = node_->create_subscription<referee_msg::msg::Referee>(
      "/Referee",
      10,
      std::bind(&IsHeadState::headStateCallback, this, std::placeholders::_1),
      options);
    callback_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    initialized_ = true;
  }

  if (!getInput("expected_state", expected_state_)) {
    throw BT::RuntimeError("IsHeadState missing input [expected_state]");
  }

  has_head_state_ = false;
  last_head_state_ = 0;
  RCLCPP_INFO(node_->get_logger(), "IsHeadState started: expected=%s", expected_state_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus IsHeadState::onRunning()
{
  callback_executor_.spin_some();
  std::lock_guard<std::mutex> lock(mutex_);

  auto clock = node_->get_clock();
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *clock,
    2000,
    "IsHeadState onRunning: expected=%s, has_state=%d, last_head=%d",
    expected_state_.c_str(), has_head_state_, last_head_state_);

  if (!has_head_state_) {
    return BT::NodeStatus::RUNNING;
  }

  if (expected_state_ == "DOWN_DONE") {
    if (last_head_state_ == 1) {
      RCLCPP_INFO(node_->get_logger(), "IsHeadState DOWN_DONE success");
      return BT::NodeStatus::SUCCESS;
    }
    if (last_head_state_ == static_cast<int8_t>(-1)) {
      RCLCPP_ERROR(node_->get_logger(), "IsHeadState DOWN_DONE failed");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (expected_state_ == "UP_DONE") {
    if (last_head_state_ == 2) {
      RCLCPP_INFO(node_->get_logger(), "IsHeadState UP_DONE success");
      return BT::NodeStatus::SUCCESS;
    }
    if (last_head_state_ == static_cast<int8_t>(-1)) {
      RCLCPP_ERROR(node_->get_logger(), "IsHeadState UP_DONE failed");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  throw BT::RuntimeError("IsHeadState expected_state must be DOWN_DONE or UP_DONE");
}

void IsHeadState::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "IsHeadState halted while waiting expected=%s, has_state=%d, last_head=%d",
    expected_state_.c_str(),
    has_head_state_,
    last_head_state_);
}

// ------------------------------------------------------------------------ //

SwitchRouteMode::SwitchRouteMode(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<topology_global_planner::srv::SwitchRouteMode>(service_node_name, conf)
{
}

BT::PortsList SwitchRouteMode::providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<std::string>(
        "service_name",
        std::string("TopoPlanner/switch_route_mode"),
        "Topology switch route mode service name"),
      BT::InputPort<std::string>("connector_id")
    });
}

void SwitchRouteMode::on_tick()
{
  request_->connector_id.clear();

  if (!getInput("connector_id", request_->connector_id)) {
    RCLCPP_ERROR(node_->get_logger(), "SwitchRouteMode missing input port 'connector_id'");
    return;
  }

  if (request_->connector_id.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "SwitchRouteMode got empty connector_id");
    return;
  }

  RCLCPP_WARN(node_->get_logger(), "SwitchRouteMode request: connector_id='%s'", request_->connector_id.c_str());
}

BT::NodeStatus SwitchRouteMode::on_completion(
  std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Response> response)
{
  if (!response->success) {
    RCLCPP_WARN(node_->get_logger(), "SwitchRouteMode failed");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_WARN(node_->get_logger(), "SwitchRouteMode success");
  return BT::NodeStatus::SUCCESS;
}

// ------------------------------------------------------------------------ //

ReConnectorCost::ReConnectorCost(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<topology_global_planner::srv::RestoreConnectorCost>(
    service_node_name, conf)
{
}

BT::PortsList ReConnectorCost::providedPorts()
{
  return providedBasicPorts({});
}

void ReConnectorCost::on_tick()
{
  request_ = std::make_shared<topology_global_planner::srv::RestoreConnectorCost::Request>();
  RCLCPP_WARN(node_->get_logger(), "ReConnectorCost request");
}

BT::NodeStatus ReConnectorCost::on_completion(
  std::shared_ptr<topology_global_planner::srv::RestoreConnectorCost::Response> response)
{
  if (response->success) {
    RCLCPP_WARN(node_->get_logger(), "ReConnectorCost success");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(node_->get_logger(), "ReConnectorCost failed");
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::QueryTopologyRoute>("QueryTopologyRoute");
  factory.registerNodeType<nav2_behavior_tree::IsConnectorActionRequired>("IsConnectorActionRequired");
  factory.registerNodeType<nav2_behavior_tree::SetConnectorStage>("SetConnectorStage");
  factory.registerNodeType<nav2_behavior_tree::SetHeadCommand>("SetHeadCommand");
  factory.registerNodeType<nav2_behavior_tree::ConnectorTransactionGuard>("ConnectorTransactionGuard");
  factory.registerNodeType<nav2_behavior_tree::ExecuteConnectorAction>("ExecuteConnectorAction");
  factory.registerNodeType<nav2_behavior_tree::IsHeadState>("IsHeadState");
  factory.registerNodeType<nav2_behavior_tree::SwitchRouteMode>("SwitchRouteMode");
  factory.registerNodeType<nav2_behavior_tree::ReConnectorCost>("ReConnectorCost");
}
