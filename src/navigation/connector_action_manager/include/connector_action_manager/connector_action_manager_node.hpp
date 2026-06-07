#pragma once

#include <mutex>
#include <string>
#include <unordered_set>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "topology_global_planner/srv/execute_connector_action.hpp"
#include "referee_msg/msg/referee.hpp"

namespace connector_action_manager
{

enum class HeadTraversalState
{
  IDLE,
  REGISTERED,
  WAITING_FOR_DOWN,
  DOWN_DONE,
  WAITING_FOR_UP,
  UP_DONE,
  FAILED,
  ABORTING
};

class ConnectorActionManagerNode : public rclcpp::Node
{
public:
  ConnectorActionManagerNode();

private:
  using ExecuteConnectorAction = topology_global_planner::srv::ExecuteConnectorAction;
  void handleExecute(
    const std::shared_ptr<ExecuteConnectorAction::Request> request,
    std::shared_ptr<ExecuteConnectorAction::Response> response);

  void holdingPoseTimerCallback();
  void upMonitorTimerCallback();
  void headStateCallback(const referee_msg::msg::Referee::SharedPtr msg);

  bool executeBegin(
    const std::shared_ptr<ExecuteConnectorAction::Request> request,
    std::shared_ptr<ExecuteConnectorAction::Response> response);
  bool executePre(
    const std::shared_ptr<ExecuteConnectorAction::Request> request,
    std::shared_ptr<ExecuteConnectorAction::Response> response);
  bool executePassed(
    const std::shared_ptr<ExecuteConnectorAction::Request> request,
    std::shared_ptr<ExecuteConnectorAction::Response> response);
  bool executeAbort(
    const std::shared_ptr<ExecuteConnectorAction::Request> request,
    std::shared_ptr<ExecuteConnectorAction::Response> response);
  bool executeEnd(
    const std::shared_ptr<ExecuteConnectorAction::Request> request,
    std::shared_ptr<ExecuteConnectorAction::Response> response);
  bool executeForceHeadCommand(
    bool down,
    std::shared_ptr<ExecuteConnectorAction::Response> response);

  void resetSessionState();
  void publishHeadDownCommand(bool down);
  void applyCancelPolicy(const std::string & cancel_policy);
  std::string stateToString(HeadTraversalState state) const;

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr subscriber_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;

  rclcpp::Service<ExecuteConnectorAction>::SharedPtr execute_service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr head_down_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr holding_pose_pub_;
  rclcpp::Subscription<referee_msg::msg::Referee>::SharedPtr head_state_sub_;
  rclcpp::TimerBase::SharedPtr holding_pose_timer_;
  rclcpp::TimerBase::SharedPtr up_monitor_timer_;

  std::mutex mutex_;

  std::string active_session_id_;
  std::string active_connector_id_;
  HeadTraversalState state_{HeadTraversalState::IDLE};

  bool publish_holding_pose_{false};
  bool hold_head_down_after_abort_{false};
  geometry_msgs::msg::PoseStamped active_holding_pose_;

  rclcpp::Time passed_time_;
  double up_monitor_timeout_{30.0};
};

}  // namespace connector_action_manager
