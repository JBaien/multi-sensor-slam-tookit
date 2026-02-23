#ifndef LIDAR_REFLECTOR_TARGET_TRACKER_KALMAN_FILTER_H
#define LIDAR_REFLECTOR_TARGET_TRACKER_KALMAN_FILTER_H

#include <Eigen/Core>

namespace tracker
{

/**
 * @brief 恒速模型卡尔曼滤波器 (Constant Velocity Model)
 *
 * 状态向量: [x, y, vx, vy]
 * 用于跟踪目标的位置和速度
 */
class KalmanFilterCV
{
public:
  KalmanFilterCV() = default;

  /**
   * @brief 重置滤波器，用初始位置初始化
   * @param x 初始x坐标
   * @param y 初始y坐标
   */
  void reset(double x, double y);

  /**
   * @brief 检查滤波器是否已初始化
   */
  bool inited() const { return inited_; }

  /**
   * @brief 预测步骤
   * @param dt 时间间隔(秒)
   * @param q_pos 位置过程噪声参数
   * @param q_vel 速度过程噪声参数
   */
  void predict(double dt, double q_pos, double q_vel);

  /**
   * @brief 更新步骤（观测融合）
   * @param meas_x 测量的x坐标
   * @param meas_y 测量的y坐标
   * @param r_meas 测量噪声方差
   */
  void update(double meas_x, double meas_y, double r_meas);

  /**
   * @brief 获取估计的x坐标
   */
  double x() const { return x_(0); }

  /**
   * @brief 获取估计的y坐标
   */
  double y() const { return x_(1); }

  /**
   * @brief 获取估计的x方向速度
   */
  double vx() const { return x_(2); }

  /**
   * @brief 获取估计的y方向速度
   */
  double vy() const { return x_(3); }

  /**
   * @brief 获取估计的总速度大小
   */
  double speed() const { return std::hypot(x_(2), x_(3)); }

private:
  bool inited_{false};                                /// 是否已初始化
  Eigen::Vector4d x_{Eigen::Vector4d::Zero()};      /// 状态向量 [x, y, vx, vy]
  Eigen::Matrix4d P_{Eigen::Matrix4d::Identity()};  /// 协方差矩阵
};

} // namespace tracker

#endif // LIDAR_REFLECTOR_TARGET_TRACKER_KALMAN_FILTER_H
