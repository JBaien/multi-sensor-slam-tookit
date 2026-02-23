/**
 * @file kalman_filter.cpp
 * @brief 2D卡尔曼滤波器实现
 */

#include "lidar_target_localization/kalman_filter.h"

KalmanFilter2D::KalmanFilter2D()
    : initialized_(false)
{
    // 观测矩阵 H: 只观测位置，不观测速度
    // [x_obs]   [1 0 0 0]   [x     ]
    // [y_obs] = [0 1 0 0] * [y     ]
    //                           [vx    ]
    //                           [vy    ]
    H_.setZero();
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;

    // 默认噪声参数
    Q_ = Eigen::Matrix4d::Identity() * 0.01;  // 过程噪声
    R_ = Eigen::Matrix2d::Identity() * 0.05;  // 观测噪声
    P_ = Eigen::Matrix4d::Identity();          // 初始协方差
}

void KalmanFilter2D::init(double x, double y)
{
    // 初始化状态: 位置为(x,y)，速度为0
    x_ << x, y, 0.0, 0.0;
    P_ = Eigen::Matrix4d::Identity() * 1.0;  // 初始协方差矩阵
    initialized_ = true;
}

void KalmanFilter2D::predict(double dt)
{
    if (!initialized_) return;

    // 状态转移矩阵 F (匀速模型):
    // x_new = x_old + vx * dt
    // y_new = y_old + vy * dt
    // vx_new = vx_old
    // vy_new = vy_old
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    // 卡尔曼预测步骤:
    // x_pred = F * x_prev
    // P_pred = F * P_prev * F^T + Q
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q_;
}

Eigen::Vector2d KalmanFilter2D::update(double x, double y)
{
    if (!initialized_)
    {
        init(x, y);
        return Eigen::Vector2d(x, y);
    }

    // 观测向量
    Eigen::Vector2d z(x, y);

    // 实际观测值 - 预测观测值
    Eigen::Vector2d y_err = z - H_ * x_;

    // 协方差
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;

    // 卡尔曼增益
    Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();

    // 状态更新
    // x_new = x_pred + K * y_err
    x_ = x_ + K * y_err;

    // 协方差更新 (Joseph形式，保证数值稳定性)
    // P_new = (I - K*H) * P_pred
    P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;

    return x_.head<2>();
}
