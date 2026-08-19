#ifndef SIMULATED_OBSTACLE_LAYER__SIMULATED_OBSTACLE_LAYER_HPP_
#define SIMULATED_OBSTACLE_LAYER__SIMULATED_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace simulated_obstacle_layer
{

class SimulatedObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SimulatedObstacleLayer();
  ~SimulatedObstacleLayer() override;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  bool isClearable() override {return true;}
  void activate() override;
  void deactivate() override;

private:
  void centerCallback(const geometry_msgs::msg::PointStamped::SharedPtr message);
  void enabledCallback(const std_msgs::msg::Bool::SharedPtr message);
  bool transformCenter(
    const geometry_msgs::msg::PointStamped & source,
    geometry_msgs::msg::PointStamped & target) const;
  void writeCircle(double center_x, double center_y, unsigned char value);
  void touchCircle(
    double center_x, double center_y,
    double * min_x, double * min_y, double * max_x, double * max_y);

  std::string global_frame_;
  std::string center_topic_;
  std::string enabled_topic_;
  bool rolling_window_{false};
  double radius_{0.40};
  double transform_tolerance_{0.10};
  unsigned char obstacle_cost_{nav2_costmap_2d::LETHAL_OBSTACLE};

  mutable std::mutex state_mutex_;
  geometry_msgs::msg::PointStamped requested_center_;
  bool center_received_{false};
  bool obstacle_enabled_{false};

  bool last_circle_drawn_{false};
  double last_center_x_{0.0};
  double last_center_y_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr center_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_subscription_;
};

}  // namespace simulated_obstacle_layer

#endif  // SIMULATED_OBSTACLE_LAYER__SIMULATED_OBSTACLE_LAYER_HPP_
