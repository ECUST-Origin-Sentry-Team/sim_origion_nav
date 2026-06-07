#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "referee_msg/msg/referee.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "topology_global_planner/srv/execute_connector_action.hpp"
#include "topology_global_planner/srv/query_topology_route.hpp"

namespace nav2_behavior_tree
{

class QueryTopologyRoute : public BtServiceNode<topology_global_planner::srv::QueryTopologyRoute>
{
public:
  QueryTopologyRoute(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<topology_global_planner::srv::QueryTopologyRoute::Response> response) override;

private:
  using QueryTopologyRouteSrv = topology_global_planner::srv::QueryTopologyRoute;

  bool getRobotPose(
    const std::string & global_frame,
    const std::string & robot_base_frame,
    geometry_msgs::msg::PoseStamped & pose) const;

  bool prepareRequest(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
  void applyRouteOutputs(const QueryTopologyRouteSrv::Response & response);
  bool isSameGoal(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs) const;
  bool isSameStart(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs) const;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool has_cached_route_{false};
  bool has_last_start_{false};
  geometry_msgs::msg::PoseStamped last_start_;
  geometry_msgs::msg::PoseStamped last_goal_;
  QueryTopologyRouteSrv::Response cached_route_;
};

class IsConnectorActionRequired : public BT::ConditionNode
{
public:
  IsConnectorActionRequired(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetConnectorStage : public BT::StatefulActionNode
{
public:
  SetConnectorStage(
    const std::string & node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool yielded_once_{false};
};

class SetHeadCommand : public BtServiceNode<topology_global_planner::srv::ExecuteConnectorAction>
{
public:
  SetHeadCommand(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<topology_global_planner::srv::ExecuteConnectorAction::Response> response)
  override;
};

class ConnectorTransactionGuard : public BT::DecoratorNode
{
public:
  ConnectorTransactionGuard(
    const std::string & node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  using ExecuteConnectorActionSrv = topology_global_planner::srv::ExecuteConnectorAction;

  bool ensureNode();
  bool captureInputs();
  std::string getCurrentStage();
  bool isGoalPreempted();
  bool shouldDeferPreempt(const std::string & stage) const;
  std::string preemptPolicyForStage(const std::string & stage);
  std::string failurePolicyForStage(const std::string & stage) const;
  void setTransactionActive(bool active);
  void sendAbort(const std::string & reason, const std::string & policy_override = std::string());
  bool isSameGoal(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ExecuteConnectorActionSrv>::SharedPtr abort_client_;
  ExecuteConnectorActionSrv::Request abort_request_;
  geometry_msgs::msg::PoseStamped active_goal_;
  bool has_active_goal_{false};
  bool transaction_active_{false};
  bool abort_sent_{false};
  bool preempt_pending_{false};
  std::string current_stage_{"IDLE"};
};

class ExecuteConnectorAction : public BtServiceNode<topology_global_planner::srv::ExecuteConnectorAction>
{
public:
  ExecuteConnectorAction(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<topology_global_planner::srv::ExecuteConnectorAction::Response> response)
  override;

  void halt() override;

private:
  bool isSameGoal(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs) const;

  geometry_msgs::msg::PoseStamped active_goal_;
  bool has_active_goal_{false};
};

class IsHeadState : public BT::StatefulActionNode
{
public:
  IsHeadState(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void headStateCallback(const referee_msg::msg::Referee::SharedPtr msg);

  bool initialized_{false};
  std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  rclcpp::Subscription<referee_msg::msg::Referee>::SharedPtr head_state_sub_;
  int8_t last_head_state_{0};
  bool has_head_state_{false};
  std::string expected_state_;
};

}  // namespace nav2_behavior_tree
