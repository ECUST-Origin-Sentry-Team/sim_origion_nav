#pragma once
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
namespace fast_layer
{
class FastLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  FastLayer();
  virtual ~FastLayer();
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);
  void updateOrigin(double new_origin_x, double new_origin_y);
  // virtual void matchSize();
  virtual void reset();
  virtual bool isClearable() { return true; }
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void deactivate();
  virtual void activate();

protected:
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);

private:
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  std::string global_frame_;
  bool rolling_window_;
  bool was_reset_;
  bool footprint_clearing_enabled_;
  std::mutex mutex_cloud;
  rclcpp::Clock::SharedPtr clock_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};
}  // namespace fast_layer
