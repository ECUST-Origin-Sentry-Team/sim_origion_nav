#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <rm_interfaces/msg/gimbal.hpp>


using Eigen::Matrix2d;
using Eigen::Vector2d;

struct TwoStateKF {
  Vector2d x;   // [theta, bias]
  Matrix2d P;
  Matrix2d Q;
  double R_motor;
  double R_odom;

  // Constructor: Initialize state and covariance matrix
  TwoStateKF();
  
  // Initialize the Kalman Filter with given parameters
  void init(double q_angle_rad2, double q_bias_rad2, double r_motor_rad2, double r_odom_rad2);
  
  // Prediction step of the Kalman Filter
  void predict();
  
  // Update step for motor measurements: H = [1, -1]
  void updateMotor(double z);  
  
  // Update step for odom measurements: H = [1, 0]
  void updateOdom(double z);  
  
  // Normalize angles to [-pi, pi]
  static double wrap(double a);
};

class BaselinkKFNode : public rclcpp::Node {
public:
  // Constructor: Node initialization
  BaselinkKFNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

private:
  // Callback function to process gimbal status messages
  void gimbalCallback(const rm_interfaces::msg::Gimbal::SharedPtr msg);
  
  // Callback function to process IMU data from /livox/imu topic
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  
  // Timer callback for periodically querying odom tf
  void odomTimerCallback();
  
  // Publish the fused TF (yaw, pitch, roll) after Kalman filter updates
  void publishTf();

  // Kalman filters for yaw, pitch, and roll (three axes)
  TwoStateKF kf_[3];

  // TF management
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Odom data (position)
  double x = 0.0, y = 0.0, z = 0.0;

  // Subscriptions to gimbal status and IMU topics
  rclcpp::Subscription<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  // Timer for querying odom tf at regular intervals
  rclcpp::TimerBase::SharedPtr odom_timer_;

  // Node parameters
  bool gimbal_in_degrees_;
  std::string odom_frame_;   // Name of the parent frame (e.g., "odom")
  std::string base_frame_;   // Name of the base frame (e.g., "base_link")
  std::string kalman_frame_; // Name of the child frame to publish
  double odom_query_hz_;     // Frequency at which to query odom tf

  // Mutex to protect shared data across multiple threads
  std::mutex mutex_;
};
