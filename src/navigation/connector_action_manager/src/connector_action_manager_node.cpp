#include "connector_action_manager/connector_action_manager_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

using namespace std::chrono_literals;  // NOLINT

namespace connector_action_manager
{

namespace
{

constexpr int8_t HEAD_RESULT_PENDING = 0;
constexpr int8_t HEAD_RESULT_DOWN_DONE = 1;
constexpr int8_t HEAD_RESULT_UP_DONE = 2;
constexpr int8_t HEAD_RESULT_FAILED = -1;

}  // namespace

ConnectorActionManagerNode::ConnectorActionManagerNode()
: Node("connector_action_manager")
{
  service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  subscriber_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  head_down_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("/head/down_cmd", 10);
  holding_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/connector_action_manager/holding_pose", 10);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = subscriber_group_;
  head_state_sub_ = this->create_subscription<referee_msg::msg::Referee>(
    "/Referee",
    10,
    std::bind(&ConnectorActionManagerNode::headStateCallback, this, std::placeholders::_1),
    sub_options);

  holding_pose_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&ConnectorActionManagerNode::holdingPoseTimerCallback, this),
    timer_group_);
  up_monitor_timer_ = this->create_wall_timer(
    500ms,
    std::bind(&ConnectorActionManagerNode::upMonitorTimerCallback, this),
    timer_group_);

  execute_service_ = this->create_service<ExecuteConnectorAction>(
    "/connector_action_manager/execute",
    std::bind(
      &ConnectorActionManagerNode::handleExecute,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_group_);

  RCLCPP_INFO(this->get_logger(), "connector_action_manager is ready");
}
// 根据不同 phase 调用不同处理函数，最后返回执行结果和新的状态
void ConnectorActionManagerNode::handleExecute(
  const std::shared_ptr<ExecuteConnectorAction::Request> request,
  std::shared_ptr<ExecuteConnectorAction::Response> response)
{
  const std::string & phase = request->phase;

  // These phases are global head commands, not connector transactions. They are
  // intentionally accepted even when connector_id/action_type are empty. This is
  // used by the BT normal-navigation branch to explicitly publish down=false and
  // clear any previous down-hold after an aborted connector transaction.
  if (phase == "force_up") {
    executeForceHeadCommand(false, response);
    return;
  }

  if (phase == "force_down") {
    executeForceHeadCommand(true, response);
    return;
  }

  if (request->connector_id.empty()) {
    response->success = false;
    response->message = "connector_id is empty";
    response->new_state = stateToString(HeadTraversalState::FAILED);
    return;
  }

  if (request->action_type != "down") {
    response->success = false;
    response->message = "Unsupported action_type: " + request->action_type;
    response->new_state = stateToString(HeadTraversalState::FAILED);
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Execute request: phase='%s', connector='%s', session='%s', action='%s', policy='%s', timeout=%.2f",
    request->phase.c_str(),
    request->connector_id.c_str(),
    request->session_id.c_str(),
    request->action_type.c_str(),
    request->cancel_policy.c_str(),
    request->timeout);

  if (phase == "begin") {
    executeBegin(request, response);
    return;
  }

  if (phase == "pre") {
    executePre(request, response);
    return;
  }

  if (phase == "passed") {
    executePassed(request, response);
    return;
  }

  if (phase == "abort") {
    executeAbort(request, response);
    return;
  }

  if (phase == "end") {
    executeEnd(request, response);
    return;
  }

  response->success = false;
  response->message = "Unsupported phase: " + phase;
  response->new_state = stateToString(HeadTraversalState::FAILED);
}

bool ConnectorActionManagerNode::executeBegin(
  const std::shared_ptr<ExecuteConnectorAction::Request> request,
  std::shared_ptr<ExecuteConnectorAction::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);

  active_session_id_ = request->session_id.empty() ? request->connector_id : request->session_id;
  active_connector_id_ = request->connector_id;
  active_holding_pose_ = request->wait_pose;
  publish_holding_pose_ = false;
  // Do not clear hold_head_down_after_abort_ here. If this begin belongs to a
  // new reverse traversal immediately after a preempt-at-exit abort, the safest
  // behavior is to keep publishing down=true until executePre() takes ownership.
  state_ = HeadTraversalState::REGISTERED;

  response->success = true;
  response->message = "Connector traversal session registered.";
  response->new_state = stateToString(state_);
  return true;
}

bool ConnectorActionManagerNode::executePre(
  const std::shared_ptr<ExecuteConnectorAction::Request> request,
  std::shared_ptr<ExecuteConnectorAction::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_session_id_ = request->session_id.empty() ? request->connector_id : request->session_id;
    active_connector_id_ = request->connector_id;
    active_holding_pose_ = request->wait_pose;
    hold_head_down_after_abort_ = false;

    if (state_ != HeadTraversalState::WAITING_FOR_DOWN && state_ != HeadTraversalState::DOWN_DONE) {
      publish_holding_pose_ = true;
      state_ = HeadTraversalState::WAITING_FOR_DOWN;
    }
  }

  std_msgs::msg::Bool down_cmd;
  down_cmd.data = true;
  head_down_cmd_pub_->publish(down_cmd);

  response->success = true;
  response->message = "Robot head down request started.";
  response->new_state = stateToString(HeadTraversalState::WAITING_FOR_DOWN);
  return true;
}

bool ConnectorActionManagerNode::executePassed(
  const std::shared_ptr<ExecuteConnectorAction::Request> request,
  std::shared_ptr<ExecuteConnectorAction::Response> response)
{
  HeadTraversalState new_state;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    active_session_id_ = request->session_id.empty() ? request->connector_id : request->session_id;
    active_connector_id_ = request->connector_id;
    publish_holding_pose_ = false;
    hold_head_down_after_abort_ = false;
    if (state_ != HeadTraversalState::WAITING_FOR_UP && state_ != HeadTraversalState::UP_DONE) {
      state_ = HeadTraversalState::WAITING_FOR_UP;
      passed_time_ = this->now();
      up_monitor_timeout_ = request->timeout > 0.0 ? request->timeout : 30.0;
    }
    new_state = state_;
  }

  // Through the connector: actively request head-up. The old version only switched
  // to WAITING_FOR_UP, so UP_DONE could be waited forever if the lower controller
  // requires an explicit command.
  publishHeadDownCommand(false);

  response->success = true;
  response->message = "Connector passed. Robot head up request started.";
  response->new_state = stateToString(new_state);
  return true;
}

bool ConnectorActionManagerNode::executeAbort(
  const std::shared_ptr<ExecuteConnectorAction::Request> request,
  std::shared_ptr<ExecuteConnectorAction::Response> response)
{
  const std::string session_id = request->session_id.empty() ? request->connector_id : request->session_id;
  const std::string cancel_policy = request->cancel_policy.empty() ? "up_if_safe" : request->cancel_policy;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!active_session_id_.empty() && !session_id.empty() && active_session_id_ != session_id) {
      response->success = true;
      response->message = "Abort ignored because the active session is already different.";
      response->new_state = stateToString(state_);
      return true;
    }

    state_ = HeadTraversalState::ABORTING;
    publish_holding_pose_ = false;
    hold_head_down_after_abort_ = (cancel_policy == "down_if_safe");
    active_holding_pose_ = geometry_msgs::msg::PoseStamped();
    active_connector_id_.clear();
    active_session_id_.clear();
  }

  applyCancelPolicy(cancel_policy);

  response->success = true;
  response->message = "Connector traversal abort acknowledged. Active head session stopped immediately.";
  response->new_state = stateToString(HeadTraversalState::ABORTING);
  return true;
}

bool ConnectorActionManagerNode::executeEnd(
  const std::shared_ptr<ExecuteConnectorAction::Request> request,
  std::shared_ptr<ExecuteConnectorAction::Response> response)
{
  (void)request;

  std::lock_guard<std::mutex> lock(mutex_);
  resetSessionState();
  response->success = true;
  response->message = "Connector traversal session ended.";
  response->new_state = stateToString(state_);
  return true;
}

bool ConnectorActionManagerNode::executeForceHeadCommand(
  bool down,
  std::shared_ptr<ExecuteConnectorAction::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (down) {
      state_ = HeadTraversalState::ABORTING;
      publish_holding_pose_ = false;
      hold_head_down_after_abort_ = true;
      active_holding_pose_ = geometry_msgs::msg::PoseStamped();
      active_connector_id_.clear();
      active_session_id_.clear();
    } else {
      resetSessionState();
    }
  }

  publishHeadDownCommand(down);

  response->success = true;
  response->message = down ?
    "Forced head command: down=true." :
    "Forced head command: down=false and connector hold cleared.";
  response->new_state = down ?
    stateToString(HeadTraversalState::ABORTING) :
    stateToString(HeadTraversalState::IDLE);
  return true;
}

void ConnectorActionManagerNode::holdingPoseTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // The head-down flag is continuously published in every state. This makes the
  // command edge deterministic: normal navigation and finished sessions publish
  // false, while lowering/crossing/aborted-hold publish true.
  std_msgs::msg::Bool head_cmd;
  head_cmd.data =
    hold_head_down_after_abort_ ||
    state_ == HeadTraversalState::WAITING_FOR_DOWN ||
    state_ == HeadTraversalState::DOWN_DONE;
  head_down_cmd_pub_->publish(head_cmd);

  if (!publish_holding_pose_) {
    return;
  }

  auto pose = active_holding_pose_;
  pose.header.stamp = this->now();
  holding_pose_pub_->publish(pose);

  // head_cmd above already publishes down=true during WAITING_FOR_DOWN.
}

void ConnectorActionManagerNode::upMonitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (state_ != HeadTraversalState::WAITING_FOR_UP) {
    return;
  }

  std_msgs::msg::Bool up_cmd;
  up_cmd.data = false;
  head_down_cmd_pub_->publish(up_cmd);

  const double elapsed = (this->now() - passed_time_).seconds();
  if (elapsed > up_monitor_timeout_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Connector '%s' has not reported robot head up done after %.2f seconds.",
      active_connector_id_.c_str(),
      elapsed);
  }
}

void ConnectorActionManagerNode::headStateCallback(const referee_msg::msg::Referee::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->head_status == HEAD_RESULT_UP_DONE && !active_connector_id_.empty()) {
    if (state_ == HeadTraversalState::WAITING_FOR_UP) {
      state_ = HeadTraversalState::UP_DONE;
    }
  } else if (msg->head_status == HEAD_RESULT_DOWN_DONE && !active_connector_id_.empty()) {
    if (state_ == HeadTraversalState::WAITING_FOR_DOWN) {
      state_ = HeadTraversalState::DOWN_DONE;
      publish_holding_pose_ = false;
    }
  } else if (msg->head_status == HEAD_RESULT_FAILED) {
    state_ = HeadTraversalState::FAILED;
    publish_holding_pose_ = false;
  }
}

void ConnectorActionManagerNode::resetSessionState()
{
  active_session_id_.clear();
  active_connector_id_.clear();
  publish_holding_pose_ = false;
  hold_head_down_after_abort_ = false;
  state_ = HeadTraversalState::IDLE;
}

void ConnectorActionManagerNode::publishHeadDownCommand(bool down)
{
  std_msgs::msg::Bool cmd;
  cmd.data = down;
  head_down_cmd_pub_->publish(cmd);
  RCLCPP_INFO(this->get_logger(), "Published head command: down=%d", down);
}

void ConnectorActionManagerNode::applyCancelPolicy(const std::string & cancel_policy)
{
  if (cancel_policy == "down_if_safe") {
    publishHeadDownCommand(true);
    RCLCPP_WARN(this->get_logger(), "Abort policy down_if_safe: keeping/requesting head down.");
    return;
  }

  // Default to head-up on abort/preemption. This prevents the robot from being
  // preempted into ordinary final-goal navigation while the head is still down.
  publishHeadDownCommand(false);
  RCLCPP_WARN(
    this->get_logger(),
    "Abort policy '%s': requesting head up.",
    cancel_policy.c_str());
}

std::string ConnectorActionManagerNode::stateToString(HeadTraversalState state) const
{
  switch (state) {
    case HeadTraversalState::IDLE:
      return "IDLE";
    case HeadTraversalState::REGISTERED:
      return "REGISTERED";
    case HeadTraversalState::WAITING_FOR_DOWN:
      return "WAITING_FOR_DOWN";
    case HeadTraversalState::DOWN_DONE:
      return "DOWN_DONE";
    case HeadTraversalState::WAITING_FOR_UP:
      return "WAITING_FOR_UP";
    case HeadTraversalState::UP_DONE:
      return "UP_DONE";
    case HeadTraversalState::FAILED:
      return "FAILED";
    case HeadTraversalState::ABORTING:
      return "ABORTING";
  }

  return "UNKNOWN";
}

}  // namespace connector_action_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<connector_action_manager::ConnectorActionManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
