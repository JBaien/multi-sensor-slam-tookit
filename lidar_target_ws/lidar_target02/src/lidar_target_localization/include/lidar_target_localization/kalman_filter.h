/**
 * @file kalman_filter.h
 * @brief 2D匀速运动卡尔曼滤波器
 *
 * 实现基于常速模型(CV)的卡尔曼滤波器，用于对目标位置进行平滑滤波。
 * 状态量为 [x, y, vx, vy]，观测量为 [x, y]。
 */

#ifndef LIDAR_TARGET_LOCALIZATION_KALMAN_FILTER_H
#define LIDAR_TARGET_LOCALIZATION_KALMAN_FILTER_H

#include <Eigen/Dense>

/**
 * @class KalmanFilter2D
 * @brief 2D匀速运动卡尔曼滤波器
 *
 * 状态量: [x, y, vx, vy] - 位置和速度
 * 观测量: [x, y] - 位置
 *
 * 使用场景:
 * - 激光雷达检测到的标靶位置存在噪声
 * - 需要对检测位置进行平滑处理
 * - 预测目标下一帧的可能位置
 */
class KalmanFilter2D
{
public:
    KalmanFilter2D();
    void init(double x, double y);                                       // 初始化滤波器状态
    void predict(double dt);                                             // 状态预测
    Eigen::Vector2d update(double x, double y);                          // 状态更新
    Eigen::Vector2d getState() const { return x_.head<2>(); }           // 获取当前估计位置
    bool isInitialized() const { return initialized_; }                   // 检查是否已初始化
    void setProcessNoise(double q) { Q_ = Eigen::Matrix4d::Identity() * q; }       // 设置过程噪声协方差
    void setMeasurementNoise(double r) { R_ = Eigen::Matrix2d::Identity() * r; }   // 设置观测噪声协方差

private:
    bool initialized_;                                                    // 初始化标志
    Eigen::Vector4d x_;     // 状态向量: [x, y, vx, vy]
    Eigen::Matrix4d P_;     // 状态协方差矩阵
    Eigen::Matrix4d Q_;     // 过程噪声协方差
    Eigen::Matrix2d R_;     // 观测噪声协方差
    Eigen::Matrix<double,2,4> H_; // 观测矩阵: 从状态中提取位置 [x, y]
};

#endif // LIDAR_TARGET_LOCALIZATION_KALMAN_FILTER_H
