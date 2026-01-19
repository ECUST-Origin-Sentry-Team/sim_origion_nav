#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "base_link_aft_kf/kf_data.hpp"

class RPYKFNode : public rclcpp::Node
{
public:
  RPYKFNode();

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishTF(const rclcpp::Time &stamp);

  RPYKalmanFilter kf_;
  rclcpp::Time last_imu_time_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Eigen::Vector3d position_xyz_;
  bool has_odom_ = false;
};
