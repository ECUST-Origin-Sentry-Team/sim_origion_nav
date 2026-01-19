#include "base_link_aft_kf/kf_data.hpp"

RPYKalmanFilter::RPYKalmanFilter()
{
  x_ = Eigen::VectorXd::Zero(STATE_DIM);
  P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1e-3;

  Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * 1e-5;
  Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * 1e-6;

  R_ = Eigen::MatrixXd::Identity(MEAS_DIM, MEAS_DIM) * 1e-3;

  F_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

  H_ = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
  H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
}

void RPYKalmanFilter::predict(const Eigen::Vector3d& gyro, double dt)
{
  F_.setIdentity();
  F_.block<3,3>(0,3) = -Eigen::Matrix3d::Identity() * dt;

  x_.segment<3>(0) += (gyro - x_.segment<3>(3)) * dt;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void RPYKalmanFilter::update(const Eigen::Vector3d& rpy_meas)
{
  Eigen::Vector3d y = rpy_meas - H_ * x_;
  Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ += K * y;
  P_ = (Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H_) * P_;
}

Eigen::Vector3d RPYKalmanFilter::getRPY() const
{
  return x_.segment<3>(0);
}

Eigen::Vector3d RPYKalmanFilter::getBias() const
{
  return x_.segment<3>(3);
}
