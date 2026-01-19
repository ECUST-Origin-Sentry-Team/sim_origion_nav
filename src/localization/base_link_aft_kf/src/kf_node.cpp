#include "base_link_aft_kf/kf_node.hpp"

RPYKFNode::RPYKFNode()
    : Node("base_link_aft_kf")
{
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", 200,
      std::bind(&RPYKFNode::imuCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&RPYKFNode::odomCallback, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  last_imu_time_ = now();
}

void RPYKFNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  rclcpp::Time current_time(msg->header.stamp);
  double dt = (current_time - last_imu_time_).seconds();
  last_imu_time_ = current_time;
  if (dt <= 0.0)
    return;
  last_imu_time_ = msg->header.stamp;

  Eigen::Vector3d gyro(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);

  kf_.predict(gyro, dt);
  publishTF(msg->header.stamp);
}

void RPYKFNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 1. 提取位置
  position_xyz_.x() = msg->pose.pose.position.x;
  position_xyz_.y() = msg->pose.pose.position.y;
  position_xyz_.z() = msg->pose.pose.position.z;
  has_odom_ = true;

  // 2. 提取姿态（用于 KF 更新）
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );

  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);

  kf_.update(Eigen::Vector3d(r, p, y));
}

void RPYKFNode::publishTF(const rclcpp::Time& stamp)
{
  if (!has_odom_) return;

  Eigen::Vector3d rpy = kf_.getRPY();
  tf2::Quaternion q;
  q.setRPY(rpy.x(), rpy.y(), rpy.z());

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_link";

  // ===== XYZ 来自里程计 =====
  tf.transform.translation.x = position_xyz_.x();
  tf.transform.translation.y = position_xyz_.y();
  tf.transform.translation.z = position_xyz_.z();

  // ===== 姿态来自 KF =====
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RPYKFNode>());
  rclcpp::shutdown();
  return 0;
}
