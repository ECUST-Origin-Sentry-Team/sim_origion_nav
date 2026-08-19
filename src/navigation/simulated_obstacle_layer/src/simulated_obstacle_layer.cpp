#include "simulated_obstacle_layer/simulated_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace simulated_obstacle_layer
{

SimulatedObstacleLayer::SimulatedObstacleLayer() = default;
SimulatedObstacleLayer::~SimulatedObstacleLayer() = default;

void SimulatedObstacleLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "center_topic", rclcpp::ParameterValue("/simulated_obstacle/center"));
  declareParameter(
    "enabled_topic", rclcpp::ParameterValue("/simulated_obstacle/enabled"));
  declareParameter("radius", rclcpp::ParameterValue(0.40));
  declareParameter("cost", rclcpp::ParameterValue(254));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.10));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock the costmap node");
  }

  int configured_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".center_topic", center_topic_);
  node->get_parameter(name_ + ".enabled_topic", enabled_topic_);
  node->get_parameter(name_ + ".radius", radius_);
  node->get_parameter(name_ + ".cost", configured_cost);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);

  if (!std::isfinite(radius_) || radius_ <= 0.0) {
    throw std::invalid_argument(name_ + ".radius must be finite and greater than zero");
  }
  if (configured_cost < 1 || configured_cost > nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw std::invalid_argument(name_ + ".cost must be in [1, 254]");
  }
  if (!std::isfinite(transform_tolerance_) || transform_tolerance_ < 0.0) {
    throw std::invalid_argument(
            name_ + ".transform_tolerance must be finite and non-negative");
  }
  if (center_topic_.empty() || enabled_topic_.empty()) {
    throw std::invalid_argument(name_ + " topic names must not be empty");
  }

  obstacle_cost_ = static_cast<unsigned char>(configured_cost);
  clock_ = node->get_clock();
  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();
  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  matchSize();
  resetMaps();
  current_ = true;

  rclcpp::QoS command_qos(rclcpp::KeepLast(1));
  // Volatile subscribers accept both the GUI's transient-local publisher and
  // ordinary volatile publishers such as `ros2 topic pub`.
  command_qos.reliable().durability_volatile();
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;

  center_subscription_ =
    node->create_subscription<geometry_msgs::msg::PointStamped>(
    center_topic_, command_qos,
    std::bind(
      &SimulatedObstacleLayer::centerCallback, this,
      std::placeholders::_1),
    subscription_options);
  enabled_subscription_ = node->create_subscription<std_msgs::msg::Bool>(
    enabled_topic_, command_qos,
    std::bind(
      &SimulatedObstacleLayer::enabledCallback, this,
      std::placeholders::_1),
    subscription_options);

  RCLCPP_INFO(
    logger_,
    "%s initialized in frame '%s': center=%s, enabled=%s, radius=%.3f m, cost=%d",
    name_.c_str(), global_frame_.c_str(), center_topic_.c_str(),
    enabled_topic_.c_str(), radius_, configured_cost);
}

void SimulatedObstacleLayer::centerCallback(
  const geometry_msgs::msg::PointStamped::SharedPtr message)
{
  if (message->header.frame_id.empty()) {
    RCLCPP_WARN(logger_, "Ignoring simulated obstacle center with an empty frame_id");
    return;
  }
  if (!std::isfinite(message->point.x) || !std::isfinite(message->point.y)) {
    RCLCPP_WARN(logger_, "Ignoring simulated obstacle center with non-finite x or y");
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  requested_center_ = *message;
  center_received_ = true;
}

void SimulatedObstacleLayer::enabledCallback(const std_msgs::msg::Bool::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  obstacle_enabled_ = message->data;
}

bool SimulatedObstacleLayer::transformCenter(
  const geometry_msgs::msg::PointStamped & source,
  geometry_msgs::msg::PointStamped & target) const
{
  if (source.header.frame_id == global_frame_) {
    target = source;
    return true;
  }

  try {
    // The obstacle is fixed in its source frame. Using the latest transform keeps a
    // map-frame command aligned with a rolling odom-frame local costmap.
    const auto transform = tf_->lookupTransform(
      global_frame_, source.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
    tf2::doTransform(source, target, transform);
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "%s cannot transform simulated obstacle from '%s' to '%s': %s",
      name_.c_str(), source.header.frame_id.c_str(), global_frame_.c_str(),
      exception.what());
    return false;
  }
}

void SimulatedObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void)robot_yaw;
  useExtraBounds(min_x, min_y, max_x, max_y);

  geometry_msgs::msg::PointStamped requested_center;
  bool should_draw = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    requested_center = requested_center_;
    should_draw = enabled_ && obstacle_enabled_ && center_received_;
  }

  geometry_msgs::msg::PointStamped transformed_center;
  const bool transform_succeeded =
    should_draw && transformCenter(requested_center, transformed_center);

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_) {
    updateOrigin(
      robot_x - getSizeInMetersX() / 2.0,
      robot_y - getSizeInMetersY() / 2.0);
  }

  if (last_circle_drawn_) {
    writeCircle(last_center_x_, last_center_y_, default_value_);
    touchCircle(last_center_x_, last_center_y_, min_x, min_y, max_x, max_y);
  }

  if (transform_succeeded) {
    last_center_x_ = transformed_center.point.x;
    last_center_y_ = transformed_center.point.y;
    last_circle_drawn_ = true;
  } else if (!should_draw) {
    last_circle_drawn_ = false;
  }
  // If TF fails temporarily, retain and redraw the last valid obstacle instead
  // of making a known obstacle disappear for a cycle.

  if (last_circle_drawn_) {
    writeCircle(last_center_x_, last_center_y_, obstacle_cost_);
    touchCircle(last_center_x_, last_center_y_, min_x, min_y, max_x, max_y);
  }
  current_ = true;
}

void SimulatedObstacleLayer::writeCircle(
  double center_x, double center_y, unsigned char value)
{
  int min_mx;
  int min_my;
  int max_mx;
  int max_my;
  worldToMapEnforceBounds(center_x - radius_, center_y - radius_, min_mx, min_my);
  worldToMapEnforceBounds(center_x + radius_, center_y + radius_, max_mx, max_my);

  const double radius_squared = radius_ * radius_;
  for (int my = min_my; my <= max_my; ++my) {
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      double world_x;
      double world_y;
      mapToWorld(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
        world_x, world_y);
      const double dx = world_x - center_x;
      const double dy = world_y - center_y;
      if ((dx * dx + dy * dy) <= radius_squared) {
        costmap_[getIndex(
            static_cast<unsigned int>(mx),
            static_cast<unsigned int>(my))] = value;
      }
    }
  }
}

void SimulatedObstacleLayer::touchCircle(
  double center_x, double center_y,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  const double padding = radius_ + getResolution();
  touch(center_x - padding, center_y - padding, min_x, min_y, max_x, max_y);
  touch(center_x + padding, center_y + padding, min_x, min_y, max_x, max_y);
}

void SimulatedObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void SimulatedObstacleLayer::reset()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (last_circle_drawn_) {
    addExtraBounds(
      last_center_x_ - radius_ - getResolution(),
      last_center_y_ - radius_ - getResolution(),
      last_center_x_ + radius_ + getResolution(),
      last_center_y_ + radius_ + getResolution());
  }
  resetMaps();
  last_circle_drawn_ = false;
  current_ = true;
}

void SimulatedObstacleLayer::activate()
{
  current_ = true;
}

void SimulatedObstacleLayer::deactivate()
{
  current_ = false;
}

}  // namespace simulated_obstacle_layer

PLUGINLIB_EXPORT_CLASS(
  simulated_obstacle_layer::SimulatedObstacleLayer,
  nav2_costmap_2d::Layer)
