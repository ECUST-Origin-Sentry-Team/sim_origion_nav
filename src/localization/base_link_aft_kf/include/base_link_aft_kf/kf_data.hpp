#pragma once
#include <Eigen/Dense>

class RPYKalmanFilter
{
public:
  RPYKalmanFilter();

  void predict(const Eigen::Vector3d& gyro, double dt);
  void update(const Eigen::Vector3d& rpy_meas);

  Eigen::Vector3d getRPY() const;
  Eigen::Vector3d getBias() const;

private:
  static constexpr int STATE_DIM = 6;
  static constexpr int MEAS_DIM  = 3;

  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd H_;
};
