#ifndef LIDAR_REFLECTOR_TARGET_TRACKER_TYPES_H
#define LIDAR_REFLECTOR_TARGET_TRACKER_TYPES_H

#include <Eigen/Core>

namespace tracker
{

/**
 * @brief 圆拟合结果结构体
 */
struct CircleFitResult
{
  bool ok{false};             /// 拟合是否成功
  double cx{0}, cy{0}, r{0}; /// 圆心坐标和半径
  int inliers{0};             /// 内点数量
  double mean_abs_err{1e9};    /// 平均绝对误差
};

} // namespace tracker

#endif // LIDAR_REFLECTOR_TARGET_TRACKER_TYPES_H
