#include "base_link_aft_kf/kf_node.hpp"

TwoStateKF::TwoStateKF() {
  x.setZero();
  P = Matrix2d::Identity() * 1e-2;
  Q = Matrix2d::Zero();
  R_motor = 1e-4;
  R_odom = 1e-4;
}

void TwoStateKF::init(double q_angle_rad2, double q_bias_rad2, double r_motor_rad2, double r_odom_rad2) {
  Q.setZero();
  Q(0, 0) = q_angle_rad2;
  Q(1, 1) = q_bias_rad2;
  R_motor = r_motor_rad2;
  R_odom = r_odom_rad2;
}

void TwoStateKF::predict() {
  // F = I, so x unchanged
  P = P + Q;
}

double TwoStateKF::wrap(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

void TwoStateKF::updateMotor(double z) {
  // H = [1, -1]
  Eigen::Matrix<double, 1, 2> H;
  H << 1.0, -1.0;
  double zpred = (H * x)(0);
  double y = wrap(z - zpred);
  double S = (H * P * H.transpose())(0, 0) + R_motor;
  Vector2d K = P * H.transpose() / S;
  x = x + K * y;
  x(0) = wrap(x(0));
  P = (Matrix2d::Identity() - K * H) * P;
}

void TwoStateKF::updateOdom(double z) {
  // H = [1, 0]
  Eigen::Matrix<double, 1, 2> H;
  H << 1.0, 0.0;
  double zpred = (H * x)(0);
  double y = wrap(z - zpred);
  double S = (H * P * H.transpose())(0, 0) + R_odom;
  Vector2d K = P * H.transpose() / S;
  x = x + K * y;
  x(0) = wrap(x(0));
  P = (Matrix2d::Identity() - K * H) * P;
}

// ================= Node implementation =================

BaselinkKFNode::BaselinkKFNode(const rclcpp::NodeOptions &opts)
: Node("kf_base_link", opts)
{
  // Parameters (can be overridden in launch/param)
  odom_frame_ = this->declare_parameter("odom_frame", std::string("odom"));
  base_frame_ = this->declare_parameter("base_frame", std::string("aft_mapped"));
  kalman_frame_ = this->declare_parameter("kalman_frame", std::string("base_link_kalman"));
  odom_query_hz_ = this->declare_parameter("odom_query_hz", 10.0);

  double q_angle_deg = this->declare_parameter("q_angle_deg", 0.02); // deg
  double q_bias_deg = this->declare_parameter("q_bias_deg", 0.005); // deg
  double r_motor_deg = this->declare_parameter("r_motor_deg", 0.2); // deg
  double r_odom_deg = this->declare_parameter("r_odom_deg", 0.05);  // deg

  auto deg2rad = [](double d){ return d * M_PI / 180.0; };
  double q_angle = std::pow(deg2rad(q_angle_deg), 2);
  double q_bias  = std::pow(deg2rad(q_bias_deg), 2);
  double r_motor = std::pow(deg2rad(r_motor_deg), 2);
  double r_odom  = std::pow(deg2rad(r_odom_deg), 2);

  for (int i = 0; i < 3; i++) {
    kf_[i].init(q_angle, q_bias, r_motor, r_odom);
  }

  // TF buffer/listener/broadcaster
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Subscriptions
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/livox/imu", rclcpp::SensorDataQoS(),
    std::bind(&BaselinkKFNode::imuCallback, this, std::placeholders::_1)
  );

  // Timer to query odom tf
  odom_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / odom_query_hz_),
    std::bind(&BaselinkKFNode::odomTimerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "three_axis_kf started. Publishing TF: %s -> %s", base_frame_.c_str(), kalman_frame_.c_str());
}

static void quatToRPY(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tq);
  m.getRPY(roll, pitch, yaw);
}

void BaselinkKFNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  
  // 获取 IMU 角速度（单位：rad/s）
  double angular_velocity_x = msg->angular_velocity.x;
  double angular_velocity_y = msg->angular_velocity.y;
  double angular_velocity_z = msg->angular_velocity.z;

  // 假设时间间隔为固定值（或通过 ROS 时钟获取），假设 delta_t 为时间间隔（秒）
  double delta_t = 0.005;  // 可以根据实际数据的频率进行调整

  // 通过角速度积分更新姿态（简单的角度积分）
  double delta_yaw = angular_velocity_z * delta_t;
  double delta_pitch = angular_velocity_y * delta_t;
  double delta_roll = angular_velocity_x * delta_t;

  // 更新卡尔曼滤波器
  double imu_ang[3] = {delta_yaw, delta_pitch, delta_roll};
  for (int i = 0; i < 3; i++) {
    kf_[i].predict();
    kf_[i].updateOdom(imu_ang[i]);
  }

  // 发布融合后的 TF
  publishTf();
}

void BaselinkKFNode::odomTimerCallback() {
  // Try lookup odom->base_frame_ transform (parent = odom_frame_, child = base_frame_)
  geometry_msgs::msg::TransformStamped t;
  try {
    rclcpp::Time now = this->get_clock()->now();
    // Use latest available transform
    t = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
  } catch (const std::exception &ex) {
    return;
  }
  double odom_roll, odom_pitch, odom_yaw;
  x = t.transform.translation.x;
  y = t.transform.translation.y; 
  z = t.transform.translation.z; 
  quatToRPY(t.transform.rotation, odom_roll, odom_pitch, odom_yaw);

  std::lock_guard<std::mutex> lk(mutex_);
  // For each axis: predict then update with odom (H=[1,0])
  double odom_ang[3] = {odom_yaw, odom_pitch, odom_roll};
  for (int i = 0; i < 3; i++) {
    kf_[i].predict();
    kf_[i].updateOdom(odom_ang[i]);
  }
  publishTf();
}

void BaselinkKFNode::publishTf() {
  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = this->get_clock()->now();
  out.header.frame_id = "odom";
  out.child_frame_id = kalman_frame_;

  // fused roll/pitch/yaw: each kf_.x(0)
  double fused_yaw  = kf_[0].x(0);
  double fused_pitch= kf_[1].x(0);
  double fused_roll = kf_[2].x(0);

  tf2::Quaternion q;
  q.setRPY(fused_roll, fused_pitch, fused_yaw);
  out.transform.rotation.x = q.x();
  out.transform.rotation.y = q.y();
  out.transform.rotation.z = q.z();
  out.transform.rotation.w = q.w();

  out.transform.translation.x = 0.0;
  out.transform.translation.y = 0.0;
  out.transform.translation.z = 0.0;

  tf_broadcaster_->sendTransform(out);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<BaselinkKFNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
