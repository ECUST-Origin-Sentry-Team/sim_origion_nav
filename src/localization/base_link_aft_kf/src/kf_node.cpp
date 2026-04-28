#include "base_link_aft_kf/kf_node.hpp"

#include <algorithm>
#include <chrono>
#include <deque>
#include <mutex>

namespace
{
struct ImuHistoryEntry
{
  rclcpp::Time stamp;
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  double dt{0.0};
  RPYKalmanFilter kf;
};

std::deque<ImuHistoryEntry> g_imu_hist;
std::mutex g_kf_mutex;

constexpr double kHistorySec = 2.0;
constexpr double kMaxImuDtSec = 0.2;
}  // namespace

RPYKFNode::RPYKFNode()
: Node("base_link_aft_kf")
{
  this->declare_parameter<std::string>("imu_topic", "/livox/imu");
  this->declare_parameter<std::string>("odom_topic", "/odom");
  this->declare_parameter<std::string>("output_odom_topic", "/odom_kf");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("output_child_frame_id", "aft_mapped_kf");
  this->declare_parameter<double>("publish_rate_hz", 100.0);

  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("output_odom_topic", output_odom_topic_);
  this->get_parameter("odom_frame_id", odom_frame_id_);
  this->get_parameter("output_child_frame_id", output_child_frame_id_);
  this->get_parameter("publish_rate_hz", publish_rate_hz_);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 500,
      std::bind(&RPYKFNode::imuCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 100,
      std::bind(&RPYKFNode::odomCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 100);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  const double safe_rate = std::max(1.0, publish_rate_hz_);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / safe_rate));
  publish_timer_ = create_wall_timer(
      period,
      std::bind(&RPYKFNode::publishStateTimer, this));

  last_imu_time_ = now();
}

void RPYKFNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(g_kf_mutex);

  const rclcpp::Time current_time(msg->header.stamp);
  const double dt = (current_time - last_imu_time_).seconds();

  if (!(dt > 0.0) || dt > kMaxImuDtSec)
  {
    last_imu_time_ = current_time;
    g_imu_hist.clear();
    return;
  }
  last_imu_time_ = current_time;
  last_imu_stamp_ = current_time;
  has_imu_stamp_ = true;

  const Eigen::Vector3d gyro(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);

  kf_.predict(gyro, dt);

  ImuHistoryEntry e;
  e.stamp = current_time;
  e.gyro = gyro;
  e.dt = dt;
  e.kf = kf_;
  g_imu_hist.push_back(e);

  while (!g_imu_hist.empty() &&
         (current_time - g_imu_hist.front().stamp).seconds() > kHistorySec)
  {
    g_imu_hist.pop_front();
  }
}

void RPYKFNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(g_kf_mutex);

  position_xyz_.x() = msg->pose.pose.position.x;
  position_xyz_.y() = msg->pose.pose.position.y;
  position_xyz_.z() = msg->pose.pose.position.z;
  has_odom_ = true;

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);

  const Eigen::Vector3d rpy_meas(r, p, y);
  const rclcpp::Time t_meas(msg->header.stamp);

  if (g_imu_hist.empty())
  {
    kf_.update(rpy_meas);
    return;
  }

  if (t_meas >= g_imu_hist.back().stamp)
  {
    kf_.update(rpy_meas);
    g_imu_hist.back().kf = kf_;
    return;
  }

  int idx = -1;
  for (int i = static_cast<int>(g_imu_hist.size()) - 1; i >= 0; --i)
  {
    if (g_imu_hist[static_cast<size_t>(i)].stamp <= t_meas)
    {
      idx = i;
      break;
    }
  }

  if (idx < 0)
  {
    return;
  }

  kf_ = g_imu_hist[static_cast<size_t>(idx)].kf;
  kf_.update(rpy_meas);
  g_imu_hist[static_cast<size_t>(idx)].kf = kf_;

  for (size_t j = static_cast<size_t>(idx) + 1; j < g_imu_hist.size(); ++j)
  {
    kf_.predict(g_imu_hist[j].gyro, g_imu_hist[j].dt);
    g_imu_hist[j].kf = kf_;
  }
}

void RPYKFNode::publishStateTimer()
{
  std::lock_guard<std::mutex> lock(g_kf_mutex);
  if (!has_odom_)
  {
    return;
  }

  const rclcpp::Time stamp = has_imu_stamp_ ? last_imu_stamp_ : now();
  publishState(stamp);
}

void RPYKFNode::publishState(const rclcpp::Time &stamp)
{
  Eigen::Vector3d rpy = kf_.getRPY();
  tf2::Quaternion q;
  q.setRPY(rpy.x(), rpy.y(), rpy.z());

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = odom_frame_id_;
  tf_msg.child_frame_id = output_child_frame_id_;
  tf_msg.transform.translation.x = position_xyz_.x();
  tf_msg.transform.translation.y = position_xyz_.y();
  tf_msg.transform.translation.z = position_xyz_.z();
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(tf_msg);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id = output_child_frame_id_;
  odom_msg.pose.pose.position.x = position_xyz_.x();
  odom_msg.pose.pose.position.y = position_xyz_.y();
  odom_msg.pose.pose.position.z = position_xyz_.z();
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  odom_pub_->publish(odom_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RPYKFNode>());
  rclcpp::shutdown();
  return 0;
}
