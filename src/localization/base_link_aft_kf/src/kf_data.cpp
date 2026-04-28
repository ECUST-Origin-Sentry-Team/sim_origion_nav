#include "base_link_aft_kf/kf_data.hpp"

#include <cmath>

namespace
{
constexpr double kPi = 3.141592653589793238462643383279502884;

// 将角度规范化到 [-pi, pi]，避免 yaw/roll/pitch 在跨越 ±pi 时产生“跳变”，
// 否则创新项会突然出现 ~2π 的误差，导致高转速时滤波振荡/发散。
inline double normalizeAngle(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return a - kPi;
}

inline void normalizeRPYInPlace(Eigen::VectorXd& x)
{
  x(0) = normalizeAngle(x(0));
  x(1) = normalizeAngle(x(1));
  x(2) = normalizeAngle(x(2));
}

inline Eigen::Vector3d normalizeInnovation(const Eigen::Vector3d& y)
{
  return Eigen::Vector3d(
    normalizeAngle(y.x()),
    normalizeAngle(y.y()),
    normalizeAngle(y.z()));
}
}  // namespace

RPYKalmanFilter::RPYKalmanFilter()
{
  // 状态: [roll, pitch, yaw, bgx, bgy, bgz]
  x_ = Eigen::VectorXd::Zero(STATE_DIM);

  // 初始协方差：不要过小，否则初期会“锁死”，对校正不敏感。
  P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1e-2;

  // 过程噪声（这里作为“连续时间噪声强度”的基值，在 predict() 中按 dt 离散化）
  // 经验上：gyro 噪声 -> 姿态角误差增长，bias 随机游走较慢。
  Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * 1e-3;  // rad^2 / s
  Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * 1e-4;  // (rad/s)^2 / s

  // 观测噪声：原先 1e-5 过于乐观，遇到里程计滞后会强拉回“旧姿态”，导致振荡。
  // 建议先用 1e-3 起步（std≈1.8deg），再根据实际抖动/漂移调参。
  R_ = Eigen::MatrixXd::Identity(MEAS_DIM, MEAS_DIM) * 1e-3;

  F_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

  H_ = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
  H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
}

void RPYKalmanFilter::predict(const Eigen::Vector3d& gyro, double dt)
{
  if (!(dt > 0.0)) return;

  // 状态转移：rpy_{k+1} = rpy_k + (gyro - bias)*dt
  // bias_{k+1} = bias_k
  F_.setIdentity();
  F_.block<3,3>(0,3) = -Eigen::Matrix3d::Identity() * dt;

  x_.segment<3>(0) += (gyro - x_.segment<3>(3)) * dt;
  normalizeRPYInPlace(x_);

  // 将 Q_ 视作连续噪声强度，按 dt 离散化
  const Eigen::MatrixXd Qd = Q_ * dt;
  P_ = F_ * P_ * F_.transpose() + Qd;
}

void RPYKalmanFilter::update(const Eigen::Vector3d& rpy_meas)
{
  // 创新项：务必对角度差做 wrap，避免 ±pi 处跳变。
  const Eigen::Vector3d y_raw = rpy_meas - (H_ * x_).head<3>();
  const Eigen::Vector3d y = normalizeInnovation(y_raw);

  const Eigen::Matrix3d S = (H_ * P_ * H_.transpose()).topLeftCorner<3,3>() + R_.topLeftCorner<3,3>();

  // 数值稳定：不要直接 inverse()
  const Eigen::Matrix3d S_inv = S.ldlt().solve(Eigen::Matrix3d::Identity());
  const Eigen::MatrixXd K = P_ * H_.transpose() * S_inv;

  x_ += K * y;
  normalizeRPYInPlace(x_);

  // Joseph 形式，确保 P_ 对称且半正定
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  const Eigen::MatrixXd KH = K * H_;
  P_ = (I - KH) * P_ * (I - KH).transpose() + K * R_ * K.transpose();
}

Eigen::Vector3d RPYKalmanFilter::getRPY() const
{
  return x_.segment<3>(0);
}

Eigen::Vector3d RPYKalmanFilter::getBias() const
{
  return x_.segment<3>(3);
}