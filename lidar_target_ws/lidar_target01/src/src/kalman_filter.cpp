#include <cmath>
#include <Eigen/LU>
#include "tracker/kalman_filter.h"

namespace tracker
{

void KalmanFilterCV::reset(double x, double y)
{
  // 初始化状态向量：x, y置为输入值，vx, vy置为0
  x_.setZero();
  x_(0) = x;
  x_(1) = y;

  // 初始化协方差矩阵为对角矩阵
  P_.setIdentity();
  P_ *= 1.0;

  inited_ = true;
}

void KalmanFilterCV::predict(double dt, double q_pos, double q_vel)
{
  if (!inited_) return;

  // 状态转移矩阵: x(k+1) = x(k) + vx(k)*dt
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;

  // 过程噪声协方差矩阵
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = q_pos * dt * dt;  // 位置噪声
  Q(1, 1) = q_pos * dt * dt;
  Q(2, 2) = q_vel * dt;       // 速度噪声
  Q(3, 3) = q_vel * dt;

  // 预测步骤: x_ = F * x, P_ = F * P * F' + Q
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilterCV::update(double meas_x, double meas_y, double r_meas)
{
  if (!inited_) return;

  // 观测矩阵: 只观测位置x, y
  Eigen::Matrix<double, 2, 4> H;
  H.setZero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;

  // 观测噪声协方差矩阵
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * r_meas;

  // 计算卡尔曼增益
  Eigen::Vector2d z(meas_x, meas_y);            // 观测向量
  Eigen::Vector2d y = z - H * x_;             // 新息(测量残差)
  Eigen::Matrix2d S = H * P_ * H.transpose() + R;  // 新息协方差
  Eigen::Matrix<double, 4, 2> K = P_ * H.transpose() * S.inverse();  // 卡尔曼增益

  // 更新状态和协方差: x = x + K*y, P = (I - K*H)*P
  x_ = x_ + K * y;
  Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  P_ = (I - K * H) * P_;
}

} // namespace tracker
