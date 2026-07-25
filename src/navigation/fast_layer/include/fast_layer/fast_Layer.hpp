#pragma once

#include <cstdint>
#include <mutex>
#include <string>

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
/*
  example
   fast_layer:
        plugin: "fast_layer::FastLayer"
        pointcloud_topic: <robot_namespace>/rog_map/inf_occ

  */
class FastLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  FastLayer();
  ~FastLayer() override;
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void reset() override;
  bool isClearable() override {return true;}
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;
  void deactivate() override;
  void activate() override;

private:
  std::string global_frame_;
  bool rolling_window_{false};
  double pointcloud_timeout_{0.5};
  double transform_timeout_{0.1};

  std::mutex cloud_mutex_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  int64_t last_cloud_receive_time_ns_{0};
  bool cloud_received_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};
}  // namespace fast_layer
