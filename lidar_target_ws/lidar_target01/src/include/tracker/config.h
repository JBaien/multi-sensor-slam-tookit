#ifndef LIDAR_REFLECTOR_TARGET_TRACKER_CONFIG_H
#define LIDAR_REFLECTOR_TARGET_TRACKER_CONFIG_H

#include <string>

namespace tracker
{

/**
 * @brief 跟踪器配置参数
 */
struct TrackerConfig
{
  // 输入话题
  std::string input_topic{"/points_raw"};   /// 点云输入话题

  // 点云过滤参数
  double intensity_threshold{150.0};        /// 强度阈值，只保留大于该值的点
  bool use_z_filter{true};                 /// 是否启用高度过滤
  double z_min{-1.0};                      /// 最小高度(米)
  double z_max{1.0};                       /// 最大高度(米)

  // 圆参数约束
  double r_min{0.10};                      /// 最小半径(米)
  double r_max{0.15};                      /// 最大半径(米)

  // 聚类参数
  double cluster_tolerance{0.10};         /// 聚类距离容差(米)
  int min_cluster_size{25};                /// 最小聚类点数
  int max_cluster_size{2000};              /// 最大聚类点数

  // RANSAC圆拟合参数
  int ransac_iters{200};                   /// RANSAC迭代次数
  double circle_inlier_th{0.02};           /// 圆内点阈值(米)
  int min_circle_inliers{20};              /// 最小圆内点数

  // 跟踪与ROI参数
  bool enable_kf{true};                    /// 是否启用卡尔曼滤波跟踪
  int max_missed_frames{8};                /// 最大丢失帧数，超过后宣布目标丢失
  double roi_base{1.0};                    /// 基础ROI半径(米)
  double roi_vel_gain{3.0};                 /// ROI速度增益系数

  // 卡尔曼滤波噪声参数
  double kf_q_pos{0.5};                     /// 位置过程噪声
  double kf_q_vel{2.0};                     /// 速度过程噪声
  double kf_r_meas{0.05};                  /// 测量噪声

  // 可视化参数
  bool publish_marker{true};               /// 是否发布可视化标记
  double marker_z{0.0};                    /// 标记显示的z坐标
};

} // namespace tracker

#endif // LIDAR_REFLECTOR_TARGET_TRACKER_CONFIG_H
