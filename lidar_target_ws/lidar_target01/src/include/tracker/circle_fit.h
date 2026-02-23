#ifndef LIDAR_REFLECTOR_TARGET_TRACKER_CIRCLE_FIT_H
#define LIDAR_REFLECTOR_TARGET_TRACKER_CIRCLE_FIT_H

#include <vector>
#include <Eigen/Core>
#include "tracker/types.h"

namespace tracker
{

/**
 * @brief 根据三个点计算圆心和半径
 * @param a 第一个点
 * @param b 第二个点
 * @param c 第三个点
 * @param cx 输出圆心x坐标
 * @param cy 输出圆心y坐标
 * @param r 输出圆半径
 * @return 成功返回true，三点共线或数值无效返回false
 */
inline bool computeCircleFrom3Pts(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c,
    double& cx, double& cy, double& r);

/**
 * @brief 使用RANSAC算法进行2D圆拟合
 * @param pts 输入点集
 * @param iters RANSAC迭代次数
 * @param r_min 最小半径约束
 * @param r_max 最大半径约束
 * @param inlier_th 内点阈值
 * @return 最佳拟合结果
 */
CircleFitResult ransacCircle2D(
    const std::vector<Eigen::Vector2d>& pts,
    int iters,
    double r_min, double r_max,
    double inlier_th);

} // namespace tracker

#endif // LIDAR_REFLECTOR_TARGET_TRACKER_CIRCLE_FIT_H
