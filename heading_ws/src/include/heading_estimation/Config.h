#ifndef HEADING_ESTIMATION_CONFIG_H
#define HEADING_ESTIMATION_CONFIG_H

#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

namespace heading_estimation {

/**
 * @brief 姿态估计节点的配置参数
 */
struct HeadingConfig {
    // 输入/输出话题和帧
    std::string pointcloud_topic; // 输入点云话题
    std::string base_frame;       // 基坐标系帧ID
    std::string lidar_frame;      // 激光雷达帧ID
    std::string attitude_topic;   // 输出姿态话题
    std::string distances_topic;  // 输出距离话题

    // 前进轴配置
    std::string forward_axis;       // 前进方向: +X, -X, +Y, -Y
    Eigen::Vector3d forward_vector; // 前进方向向量

    // 车体尺寸(米)
    double vehicle_length; // 车体长度
    double vehicle_width;  // 车体宽度
    double track_length;   // 履带长度
    double track_width;    // 履带间距

    // 中轴线采样点(米)
    double sample_front_dist; // 中心前方距离
    double sample_back_dist;  // 中心后方距离

    // 地面平面估计
    double ground_roi_radius;       // 地面拟合ROI半径(米)
    double ground_z_percentile;     // 地面点选择的高度分位数
    int ground_inlier_min;          // 有效地面的最小内点数
    double ground_normal_threshold; // 法向与水平的最大偏差(度)
    int ground_ransac_max_iter;     // 地面RANSAC最大迭代次数
    double ground_ransac_threshold;  // 地面RANSAC内点距离阈值(米)
    double plane_inlier_threshold;  // 平面提取的内点距离阈值(米)

    // 墙面平面估计
    double wall_z_min;              // 墙面提取的最小高度(米)
    double wall_z_max;              // 墙面提取的最大高度(米)
    double wall_z_percentile_min;   // 墙面提取的最小高度分位数(0-1)
    double wall_z_percentile_max;   // 墙面提取的最大高度分位数(0-1)
    bool wall_use_percentile_filter; // 是否使用分位数高度滤波
    int wall_inlier_min;            // 有效墙面的最小内点数
    double wall_vertical_threshold; // 垂直平面的最大|n_z|(余弦阈值)
    double wall_parallel_cos;       // 平行墙验证的余弦阈值
    int wall_max_planes;            // 最大拟合墙面数量(用于筛选左右墙)
    double wall_side_cos_threshold; // 判断是否为左右墙的余弦阈值(与前进轴垂直度)
    int wall_ransac_max_iter;       // 墙面RANSAC最大迭代次数
    double wall_ransac_threshold;   // 墙面RANSAC内点距离阈值(米)
    double wall_distance_max;       // 墙距计算的最大有效距离(米)

    // 滤波参数
    double voxel_size;      // 体素滤波尺寸(米)
    int outlier_mean_k;     // 离群点移除的邻居数量
    double outlier_std_mul; // 离群点移除的标准差倍数

    // 时域平滑
    double temporal_smoothing_tau; // 指数平滑的时间常数(秒)
    double max_yaw_rate;           // 最大航向角变化率(弧度/帧)

    // 质量阈值
    double ground_max_rms; // 地面平面的最大RMS误差(米)
    double wall_max_rms;   // 墙面平面的最大RMS误差(米)

    HeadingConfig();
    void loadFromROS(ros::NodeHandle& nh);
};

} // namespace heading_estimation

#endif // HEADING_ESTIMATION_CONFIG_H
