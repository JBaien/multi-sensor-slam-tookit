#include "heading_estimation/HeadingEstimator.h"

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace heading_estimation {

// ============================================================================
// EstimationResult 实现
// ============================================================================

EstimationResult::EstimationResult()
    : roll(0.0),
      pitch(0.0),
      yaw(0.0),
      distances(4, 0.0),
      ground_valid(false),
      left_wall_valid(false),
      right_wall_valid(false),
      ground_rms(0.0),
      left_wall_rms(0.0),
      right_wall_rms(0.0),
      ground_inliers(0),
      left_wall_inliers(0),
      right_wall_inliers(0),
      confidence_high(false),
      confidence_medium(false),
      confidence_low(true)
{
}

void EstimationResult::reset()
{
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    std::fill(distances.begin(), distances.end(), 0.0);
    ground_valid = false;
    left_wall_valid = false;
    right_wall_valid = false;
    ground_rms = 0.0;
    left_wall_rms = 0.0;
    right_wall_rms = 0.0;
    ground_inliers = 0;
    left_wall_inliers = 0;
    right_wall_inliers = 0;
    confidence_high = false;
    confidence_medium = false;
    confidence_low = true;
}

// ============================================================================
// HeadingEstimator 实现
// ============================================================================

HeadingEstimator::HeadingEstimator(const HeadingConfig& config)
    : config_(config), first_frame_(true)
{
    forward_leveled_.setZero();
    corridor_axis_prev_.setZero();

    ROS_INFO("航向估计器已初始化，前进轴: %s", config_.forward_axis.c_str());
}

HeadingEstimator::~HeadingEstimator() {}

EstimationResult HeadingEstimator::processPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const ros::Time& timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);

    EstimationResult result;
    result.reset();

    if (cloud->empty()) {
        ROS_WARN_THROTTLE(2.0, "接收到空点云");
        return prev_result_;
    }

    try {
        // 步骤 1: 滤波和降采样
        size_t original_size = cloud->size();
        auto filtered_cloud = filterPointCloud(cloud);
        size_t filtered_size = filtered_cloud->size();

        ROS_INFO(
            "滤波处理点云数量: 原始=%lu, 滤波后=%lu, 保留=%.1f%%",
            original_size, filtered_size,
            original_size > 0 ? 100.0 * filtered_size / original_size : 0.0);

        if (filtered_cloud->empty()) {
            ROS_WARN_THROTTLE(2.0, "滤波后点云为空");
            return prev_result_;
        }

        // 步骤 2: 估计地面平面和姿态
        Eigen::Vector3d ground_normal(0, 0, 1);
        double ground_rms = 0.0;
        int ground_inliers = 0;

        bool ground_valid = estimateGroundAttitude(
            filtered_cloud, ground_normal, ground_rms, ground_inliers);

        result.ground_valid = ground_valid;
        result.ground_rms = ground_rms;
        result.ground_inliers = ground_inliers;

        // 步骤 3：根据地面法向量计算调平旋转
        // 约定：computeLevelRotation() 返回的是 R_ws（设备坐标 → 调平坐标）
        // 即：R_ws * ground_normal = 调平坐标系的竖直方向 z_w
        Eigen::Matrix3d R_ws;
        if (ground_valid) {
            R_ws = computeLevelRotation(ground_normal);

            // 设备姿态矩阵：R_sw (调平坐标 → 设备坐标), 求逆
            // 设备在世界中的真实姿态应使用 R_sw
            Eigen::Matrix3d R_sw = R_ws.transpose();

            // 从旋转矩阵中提取欧拉角（ZYX 顺序）
            // 数学含义：R = Rz(yaw) * Ry(pitch) * Rx(roll)
            // 返回向量顺序为：
            // ypr[0] -> yaw   （绕 Z 轴）
            // ypr[1] -> pitch （绕 Y 轴）
            // ypr[2] -> roll  （绕 X 轴）
            Eigen::Vector3d ypr = R_sw.eulerAngles(2, 1, 0);

            // 单激光雷达场景下 yaw 不可观测，这里只输出俯仰角和横滚角
            result.pitch = ypr[1] * 180.0 / M_PI; // 俯仰角Pitch
            result.roll = ypr[2] * 180.0 / M_PI;  // 横滚角Roll

            // 更新调平系前进轴（用于yaw计算和平滑）
            forward_leveled_ = R_ws * config_.forward_vector;
            forward_leveled_.z() = 0.0; // 只保留XY分量
            double f_norm = forward_leveled_.norm();
            if (f_norm > 1e-6) {
                forward_leveled_ /= f_norm;
            } else {
                ROS_WARN_THROTTLE(
                    1.0, "Forward vector projection too small: %.6f", f_norm);
            }
        } else {
            // 地面估计失败时，仅回退横滚角和俯仰角
            ROS_WARN_THROTTLE(1.0,
                              "地面平面估计失败，横滚角和俯仰角使用上一帧");
            if (!first_frame_) {
                result.roll = prev_result_.roll;
                result.pitch = prev_result_.pitch;
            }

            // 使用上一帧的 R_ws，或者使用默认值
            // 检查上一帧姿态角是否异常，避免传播错误
            double max_tilt_angle = 15.0; // 最大倾斜角阈值
            bool prev_attitude_ok = false;

            if (!first_frame_) {
                auto normalize_angle = [](double angle) -> double {
                    while (angle > 180.0) angle -= 360.0;
                    while (angle < -180.0) angle += 360.0;
                    return angle;
                };

                double roll_norm = normalize_angle(prev_result_.roll);
                double pitch_norm = normalize_angle(prev_result_.pitch);

                if (std::abs(roll_norm) <= max_tilt_angle &&
                    std::abs(pitch_norm) <= max_tilt_angle) {
                    prev_attitude_ok = true;
                } else {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "上一帧姿态异常（Roll=%.2f°, Pitch=%.2f°），"
                        "使用默认旋转矩阵",
                        prev_result_.roll, prev_result_.pitch);
                }
            }

            if (!first_frame_ && prev_attitude_ok) {
                // 从上一帧的欧拉角重建 R_ws
                double roll_rad = prev_result_.roll * M_PI / 180.0;
                double pitch_rad = prev_result_.pitch * M_PI / 180.0;
                double yaw_rad = prev_result_.yaw * M_PI / 180.0;

                Eigen::AngleAxisd roll_angle(roll_rad,
                                             Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitch_angle(pitch_rad,
                                              Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yaw_angle(yaw_rad, Eigen::Vector3d::UnitZ());

                Eigen::Matrix3d R_sw =
                    (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
                R_ws = R_sw.transpose();

                // 更新调平系前进轴
                forward_leveled_ = R_ws * config_.forward_vector;
                forward_leveled_.z() = 0.0;
                double f_norm = forward_leveled_.norm();
                if (f_norm > 1e-6) {
                    forward_leveled_ /= f_norm;
                }
            } else {
                // 第一帧且地面估计失败，或上一帧姿态异常，使用默认旋转矩阵（无旋转）
                R_ws = Eigen::Matrix3d::Identity();
                forward_leveled_ = config_.forward_vector;
                forward_leveled_.z() = 0.0;
                forward_leveled_.normalize();
                ROS_WARN_THROTTLE(1.0, "使用默认旋转矩阵（无旋转）");
            }
        }

        // 步骤 4: 估计墙面和航向角
        Eigen::Vector3d left_wall_normal, right_wall_normal;
        double left_d = 0.0, right_d = 0.0;
        double left_rms = 0.0, right_rms = 0.0;
        int left_inliers = 0, right_inliers = 0;

        bool wall_valid = estimateWallYaw(
            filtered_cloud, R_ws, left_wall_normal, right_wall_normal, left_d,
            right_d, left_rms, right_rms, left_inliers, right_inliers);

        ROS_INFO("墙面估计状态: valid=%d, left_inliers=%d, right_inliers=%d",
                 wall_valid, left_inliers, right_inliers);
        if (!left_wall_normal.isZero()) {
            ROS_INFO("左墙法向量: [%.3f, %.3f, %.3f], d=%.3f",
                     left_wall_normal.x(), left_wall_normal.y(),
                     left_wall_normal.z(), left_d);
        }
        if (!right_wall_normal.isZero()) {
            ROS_INFO("右墙法向量: [%.3f, %.3f, %.3f], d=%.3f",
                     right_wall_normal.x(), right_wall_normal.y(),
                     right_wall_normal.z(), right_d);
        }
        ROS_INFO("前进轴: [%.3f, %.3f, %.3f]", forward_leveled_.x(),
                 forward_leveled_.y(), forward_leveled_.z());

        // 使用加权最小二乘优化
        result.right_wall_valid = (right_inliers >= config_.wall_inlier_min);
        result.left_wall_rms = left_rms;
        result.right_wall_rms = right_rms;
        result.left_wall_inliers = left_inliers;
        result.right_wall_inliers = right_inliers;

        if (wall_valid) {
            // Compute yaw from wall normals
            result.yaw =
                computeYawFromWalls(left_wall_normal, right_wall_normal,
                                    left_inliers, right_inliers);

            // 步骤 5: 计算四面墙距
            computeWallDistances(left_wall_normal, right_wall_normal, left_d,
                                 right_d, R_ws, result.distances);
        } else {
            ROS_WARN_THROTTLE(1.0, "墙面平面估计失败，航向角和距离使用上一帧");
            if (!first_frame_) {
                result.yaw = prev_result_.yaw;
                result.distances = prev_result_.distances;
            }
        }

        // 步骤 6: 应用时域平滑
        if (!first_frame_) {
            double dt = (timestamp - prev_timestamp_).toSec();
            applyTemporalSmoothing(dt, result);
        }

        // 步骤 7: 质量检查（仅用于置信度评估，不再回退所有结果）
        result.confidence_high = checkQuality(result);

        // 根据地面和墙面的有效性分别判断置信度
        if (!result.confidence_high) {
            // 质量检查失败（如姿态角异常），标记为低置信度
            result.confidence_low = true;
            result.confidence_medium = false;
        } else {
            // 质量检查通过时，根据地面和墙面的有效性判断置信度
            bool ground_ok = result.ground_valid;
            bool wall_ok = result.left_wall_valid || result.right_wall_valid;

            if (ground_ok && wall_ok) {
                result.confidence_low = false;
                result.confidence_medium = false;
            } else if (ground_ok || wall_ok) {
                result.confidence_low = false;
                result.confidence_medium = true;
            } else {
                result.confidence_low = true;
                result.confidence_medium = false;
            }
        }

        // Update previous result
        prev_result_ = result;
        prev_timestamp_ = timestamp;
        first_frame_ = false;

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in processPointCloud: %s", e.what());
        if (!first_frame_) {
            result = prev_result_;
        }
    }

    return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr HeadingEstimator::filterPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 体素网格降采样
    filtered = PointCloudProcessing::voxelFilter(cloud, config_.voxel_size);

    // 离群点移除
    filtered = PointCloudProcessing::outlierRemoval(
        filtered, config_.outlier_mean_k, config_.outlier_std_mul);

    return filtered;
}

bool HeadingEstimator::estimateGroundAttitude(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    Eigen::Vector3d& ground_normal, double& rms_error, int& inlier_count)
{
    // 地面估计 ROI 滤波
    auto roi_cloud =
        PointCloudProcessing::roiFilter(cloud, config_.ground_roi_radius);

    if (roi_cloud->size() < config_.ground_inlier_min) {
        ROS_DEBUG("ROI Points < inlier_min: %lu", roi_cloud->size());
        ROS_WARN_THROTTLE(2.0, "地面ROI点数不足: %lu < %d", roi_cloud->size(),
                          config_.ground_inlier_min);
        return false;
    }

    // 基于 Z 分位数提取低点(地面)
    std::vector<float> z_values;
    z_values.reserve(roi_cloud->size());
    for (const auto& pt : roi_cloud->points) {
        if (std::isfinite(pt.z)) {
            z_values.push_back(pt.z);
        }
    }

    if (z_values.empty()) {
        return false;
    }

    // nth_element: 只有第 k 位 位置是正确的, 其余无序
    std::nth_element(
        z_values.begin(),
        z_values.begin() +
            static_cast<size_t>(z_values.size() * config_.ground_z_percentile),
        z_values.end());
    float z_threshold = z_values[static_cast<size_t>(
        z_values.size() * config_.ground_z_percentile)];

    // 保留低于阈值的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : roi_cloud->points) {
        if (std::isfinite(pt.z) && pt.z < z_threshold) {
            ground_cloud->points.push_back(pt);
        }
    }

    if (ground_cloud->size() < config_.ground_inlier_min) {
        ROS_DEBUG("Ground points < inlier_min: %lu", ground_cloud->size());
        ROS_WARN_THROTTLE(2.0, "地面候选点数不足: %lu < %d (z_threshold=%.3f)",
                          ground_cloud->size(), config_.ground_inlier_min,
                          z_threshold);
        return false;
    }

    // RANSAC 平面拟合
    double d;
    bool success = PointCloudProcessing::ransacPlane(
        ground_cloud, ground_normal, d, rms_error, inlier_count,
        config_.ground_ransac_max_iter, config_.ground_ransac_threshold);

    if (!success || inlier_count < config_.ground_inlier_min) {
        ROS_DEBUG("Ground plane RANSAC failed: inliers=%d", inlier_count);
        ROS_WARN_THROTTLE(
            2.0, "地面RANSAC失败: success=%d, inliers=%d/%d, rms=%.3f", success,
            inlier_count, config_.ground_inlier_min, rms_error);
        return false;
    }

    // 使用加权最小二乘优化
    auto inliers = PointCloudProcessing::extractPlaneInliers(
        ground_cloud, ground_normal, d, config_.plane_inlier_threshold);
    auto weights =
        PointCloudProcessing::computeTukeyWeights(inliers, ground_normal, d);
    PointCloudProcessing::refinePlaneWeighted(inliers, ground_normal, d,
                                              weights);

    // 重新计算 RMS
    rms_error =
        PointCloudProcessing::computePlaneRMS(inliers, ground_normal, d);

    // 检查地面是否近似水平
    double z_component = std::abs(ground_normal.z());
    if (z_component <
        std::cos(config_.ground_normal_threshold * M_PI / 180.0)) {
        ROS_DEBUG("Ground plane not horizontal enough: |n_z|=%.3f",
                  z_component);
        return false;
    }

    // 确保法向量向上
    if (ground_normal.z() < 0) {
        ground_normal = -ground_normal;
    }

    return true;
}

Eigen::Matrix3d HeadingEstimator::computeLevelRotation(
    const Eigen::Vector3d& ground_normal)
{
    const Eigen::Vector3d z_w(0.0, 0.0, 1.0);

    Eigen::Vector3d n = ground_normal;
    const double n_norm = n.norm();
    if (n_norm < 1e-6) {
        return Eigen::Matrix3d::Identity();
    }
    n /= n_norm;

    // 统一法向朝向，避免 ±n 引起 180° 跳变
    if (n.dot(z_w) < 0.0) {
        n = -n;
    }

    // cos 夹紧到 [-1, 1]
    double cos_angle = n.dot(z_w);
    if (cos_angle > 1.0) cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;

    // 已经对齐
    if (std::abs(cos_angle - 1.0) < 1e-6) {
        return Eigen::Matrix3d::Identity();
    }

    // 近似反向：需要稳定轴
    if (std::abs(cos_angle + 1.0) < 1e-6) {
        Eigen::Vector3d axis = n.cross(Eigen::Vector3d::UnitX());
        if (axis.norm() < 1e-6) {
            axis = n.cross(Eigen::Vector3d::UnitY());
        }
        axis.normalize();
        return Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
    }

    // 一般情况
    Eigen::Vector3d axis = n.cross(z_w);
    const double sin_angle = axis.norm();
    axis /= sin_angle;

    const double angle = std::atan2(sin_angle, cos_angle);
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

bool HeadingEstimator::estimateWallYaw(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Matrix3d& R_level, Eigen::Vector3d& left_wall_normal,
    Eigen::Vector3d& right_wall_normal, double& left_d, double& right_d,
    double& left_rms, double& right_rms, int& left_inliers, int& right_inliers)
{
    // 初始化输出
    left_d = 0.0;
    right_d = 0.0;
    left_wall_normal.setZero();
    right_wall_normal.setZero();
    left_rms = 0.0;
    right_rms = 0.0;
    left_inliers = 0;
    right_inliers = 0;

    // 变换到调平坐标系
    auto leveled_cloud =
        PointCloudProcessing::transformPointCloud(cloud, R_level);

    // 墙面高度滤波（根据配置选择分位数或绝对高度）
    pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered;
    if (config_.wall_use_percentile_filter) {
        height_filtered = PointCloudProcessing::passThroughZPercentile(
            leveled_cloud, config_.wall_z_percentile_min,
            config_.wall_z_percentile_max);
        ROS_DEBUG("墙面高度滤波使用分位数: [%.2f, %.2f]",
                  config_.wall_z_percentile_min, config_.wall_z_percentile_max);
    } else {
        height_filtered = PointCloudProcessing::passThroughZ(
            leveled_cloud, config_.wall_z_min, config_.wall_z_max);
        ROS_DEBUG("墙面高度滤波使用绝对值: [%.2f, %.2f]m", config_.wall_z_min,
                  config_.wall_z_max);
    }

    if (height_filtered->size() < config_.wall_inlier_min) {
        ROS_DEBUG("墙面点云数量不足: %lu", height_filtered->size());
        return false;
    }

    // 1. 拟合多个垂直平面（最多 wall_max_planes 个）
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining = height_filtered;
    std::vector<Eigen::Vector3d> all_normals;
    std::vector<double> all_d_values;
    std::vector<double> all_rms_values;
    std::vector<int> all_inlier_counts;

    for (int i = 0; i < config_.wall_max_planes; ++i) {
        if (remaining->size() < config_.wall_inlier_min) {
            ROS_DEBUG("Plane %d: insufficient remaining points: %lu", i,
                      remaining->size());
            break;
        }

        Eigen::Vector3d normal;
        double d, rms;
        int inliers;
        bool success = PointCloudProcessing::ransacPlane(
            remaining, normal, d, rms, inliers, 300, 0.05);

        if (!success || inliers < config_.wall_inlier_min) {
            ROS_DEBUG("Plane %d: RANSAC failed or insufficient inliers", i);
            break;
        }

        // 井下环境可能有顶板残余、设备反射等，先被拟合到
        // 需要继续搜索真正的墙面
        if (!PointCloudProcessing::isVerticalPlane(
                normal, config_.wall_vertical_threshold)) {
            ROS_DEBUG("Plane %d not vertical: n_z=%.3f, skipping...", i,
                      normal.z());
            // 提取外点继续搜索
            remaining = PointCloudProcessing::extractPlaneOutliers(
                remaining, normal, d, config_.plane_inlier_threshold);
            continue;
        }

        all_normals.push_back(normal);
        all_d_values.push_back(d);
        all_rms_values.push_back(rms);
        all_inlier_counts.push_back(inliers);

        ROS_DEBUG("Plane %d: normal=[%.3f,%.3f,%.3f], d=%.3f, inliers=%d", i,
                  normal.x(), normal.y(), normal.z(), d, inliers);

        // 提取内点和外点，用外点继续拟合下一个平面
        remaining = PointCloudProcessing::extractPlaneOutliers(
            remaining, normal, d, config_.plane_inlier_threshold);
    }

    if (all_normals.empty()) {
        ROS_DEBUG("No valid vertical planes found");
        return false;
    }

    // 2. 根据前进轴筛选左右墙面
    // 原理：左右墙的法向量应垂直于前进轴（点积接近0）
    // 前后墙的法向量应平行或反平行于前进轴（点积接近±1）
    std::vector<Eigen::Vector3d> side_normals;
    std::vector<double> side_d_values;
    std::vector<double> side_rms_values;
    std::vector<int> side_inlier_counts;

    Eigen::Vector3d forward_leveled = forward_leveled_;

    // 安全检查
    if (forward_leveled.norm() < 1e-6) {
        ROS_WARN(
            "Forward vector projection too small, cannot filter side walls");
        return false;
    }

    for (size_t i = 0; i < all_normals.size(); ++i) {
        // 平面法向量只取XY分量（已经是垂直平面）
        Eigen::Vector3d normal_xy = all_normals[i];
        normal_xy.z() = 0.0;
        normal_xy.normalize();

        // 计算法向量与前进轴的点积
        double dot_with_forward = std::abs(normal_xy.dot(forward_leveled));

        ROS_DEBUG("Plane %zu: normal_xy=[%.3f,%.3f], dot_with_forward=%.3f", i,
                  normal_xy.x(), normal_xy.y(), dot_with_forward);

        // 判断是否为左右墙：与前进轴的点积应该很小（接近垂直）
        if (dot_with_forward < config_.wall_side_cos_threshold) {
            side_normals.push_back(all_normals[i]);
            side_d_values.push_back(all_d_values[i]);
            side_rms_values.push_back(all_rms_values[i]);
            side_inlier_counts.push_back(all_inlier_counts[i]);
            ROS_DEBUG("Side wall %zu: dot_with_forward=%.3f < %.3f (ACCEPTED)",
                      i, dot_with_forward, config_.wall_side_cos_threshold);
        } else {
            ROS_DEBUG(
                "Front/back wall %zu: dot_with_forward=%.3f >= %.3f (REJECTED)",
                i, dot_with_forward, config_.wall_side_cos_threshold);
        }
    }

    if (side_normals.empty()) {
        ROS_DEBUG(
            "No side walls (left/right) found, only front/back walls detected");
        return false;
    }

    // 3. 从筛选出的左右墙中选择最优的两个（如果有多个的话）
    // 策略：选择内点数最多的两个平行墙面
    if (side_normals.size() > 2) {
        // 按内点数排序
        std::vector<size_t> indices(side_normals.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(),
                  [&side_inlier_counts](size_t a, size_t b) {
                      return side_inlier_counts[a] > side_inlier_counts[b];
                  });

        // 选择前两个，并检查平行性
        size_t i1 = indices[0];
        size_t i2 = indices[1];
        double dot = std::abs(side_normals[i1].dot(side_normals[i2]));

        if (dot >= config_.wall_parallel_cos) {
            // 两个最平行的墙面
            side_normals = {side_normals[i1], side_normals[i2]};
            side_d_values = {side_d_values[i1], side_d_values[i2]};
            side_rms_values = {side_rms_values[i1], side_rms_values[i2]};
            side_inlier_counts = {side_inlier_counts[i1],
                                  side_inlier_counts[i2]};
        } else {
            // 尝试找平行的一对
            bool found_pair = false;
            for (size_t j = 0; j < indices.size() && !found_pair; ++j) {
                for (size_t k = j + 1; k < indices.size(); ++k) {
                    if (std::abs(side_normals[indices[j]].dot(
                            side_normals[indices[k]])) >=
                        config_.wall_parallel_cos) {
                        side_normals = {side_normals[indices[j]],
                                        side_normals[indices[k]]};
                        side_d_values = {side_d_values[indices[j]],
                                         side_d_values[indices[k]]};
                        side_rms_values = {side_rms_values[indices[j]],
                                           side_rms_values[indices[k]]};
                        side_inlier_counts = {side_inlier_counts[indices[j]],
                                              side_inlier_counts[indices[k]]};
                        found_pair = true;
                        break;
                    }
                }
            }
            if (!found_pair) {
                // 没有找到平行的墙面，只保留内点数最多的一个
                ROS_DEBUG("No parallel side walls found, keeping only one");
                side_normals = {side_normals[i1]};
                side_d_values = {side_d_values[i1]};
                side_rms_values = {side_rms_values[i1]};
                side_inlier_counts = {side_inlier_counts[i1]};
            }
        }
    }

    // 4. 分类为左墙和右墙
    Eigen::Vector3d left_leveled =
        Eigen::Vector3d(0, 0, 1).cross(forward_leveled);
    left_leveled.normalize();

    for (size_t i = 0; i < side_normals.size(); ++i) {
        if (isLeftWall(side_normals[i], side_d_values[i], left_leveled)) {
            // 如果已经有一个左墙，保留内点数更多的
            if (side_inlier_counts[i] > left_inliers) {
                left_wall_normal = side_normals[i];
                left_d = side_d_values[i];
                left_rms = side_rms_values[i];
                left_inliers = side_inlier_counts[i];
            }
        } else {
            // 如果已经有一个右墙，保留内点数更多的
            if (side_inlier_counts[i] > right_inliers) {
                right_wall_normal = side_normals[i];
                right_d = side_d_values[i];
                right_rms = side_rms_values[i];
                right_inliers = side_inlier_counts[i];
            }
        }
    }

    // 5. 检查两面墙的平行性（如果两面墙都存在）
    if (left_inliers > 0 && right_inliers > 0) {
        double parallel_dot = std::abs(left_wall_normal.dot(right_wall_normal));
        if (parallel_dot < config_.wall_parallel_cos) {
            ROS_DEBUG("Left and right walls not parallel: dot=%.3f < %.3f",
                      parallel_dot, config_.wall_parallel_cos);
            // 保留内点数更多的墙面
            if (left_inliers > right_inliers) {
                right_inliers = 0;
                right_wall_normal.setZero();
                right_d = 0.0;
                right_rms = 0.0;
            } else {
                left_inliers = 0;
                left_wall_normal.setZero();
                left_d = 0.0;
                left_rms = 0.0;
            }
        }
    }

    ROS_DEBUG("Final result: left=%d inliers, right=%d inliers", left_inliers,
              right_inliers);

    return (left_inliers > 0 || right_inliers > 0);
}

bool HeadingEstimator::isLeftWall(const Eigen::Vector3d& wall_normal,
                                  double wall_d,
                                  const Eigen::Vector3d& forward_leveled)
{
    // 计算平面上到原点最近的点: p0 = -d * n
    Eigen::Vector3d p0 = -wall_d * wall_normal;

    // 在调平坐标系中构造左轴：Z轴 × 前进轴
    Eigen::Vector3d left_leveled =
        Eigen::Vector3d(0, 0, 1).cross(forward_leveled);
    left_leveled.normalize();

    // 判断墙面在左轴上的投影是否为正
    double projection = p0.dot(left_leveled);

    return projection > 0;
}

double HeadingEstimator::computeYawFromWalls(
    const Eigen::Vector3d& left_normal, const Eigen::Vector3d& right_normal,
    int left_inliers, int right_inliers)
{
    Eigen::Vector3d corridor_axis(0, 0, 0);
    double total_weight = 0.0;

    // 检查是否有有效的墙面数据
    if (left_inliers < config_.wall_inlier_min &&
        right_inliers < config_.wall_inlier_min) {
        ROS_WARN("No valid wall data for yaw computation, using previous yaw");
        return prev_result_.yaw; // 返回上一帧 yaw 避免跳变
    }

    // 确定符号对齐的参考方向
    Eigen::Vector3d ref_axis;
    if (first_frame_ || corridor_axis_prev_.norm() < 1e-6) {
        // 第一帧或无历史数据：使用前进轴旋转 90° 作为初始参考
        ref_axis = forward_leveled_.cross(Eigen::Vector3d(0, 0, 1));
        ref_axis.normalize();
        ROS_DEBUG("Using initial reference axis for alignment: [%.3f, %.3f]",
                  ref_axis.x(), ref_axis.y());
    } else {
        // 使用上一帧的走廊轴作为参考（更稳定）
        ref_axis = corridor_axis_prev_;
        ROS_DEBUG("Using previous corridor axis for alignment: [%.3f, %.3f]",
                  ref_axis.x(), ref_axis.y());
    }

    // 左墙法向量转换为走廊轴向量（在调平坐标系内）
    // 走廊轴向量 = Z轴 × 墙面法向量（沿走廊方向的单位向量）
    if (left_inliers >= config_.wall_inlier_min) {
        Eigen::Vector3d t_left = Eigen::Vector3d(0, 0, 1).cross(left_normal);
        t_left.z() = 0.0; // 只保留XY分量（在调平坐标系内）
        double norm = t_left.norm();
        if (norm > 1e-6) {
            t_left /= norm; // 归一化
        } else {
            ROS_WARN("Left wall corridor axis norm is too small: %.6f", norm);
            return prev_result_.yaw; // 返回上一帧避免跳变
        }

        // 符号对齐：确保与参考方向一致 内积
        if (t_left.dot(ref_axis) < 0) {
            t_left = -t_left;
            ROS_DEBUG("Left wall corridor axis flipped for alignment");
        }

        // 加权融合：内点数越多，权重越大
        double weight = static_cast<double>(left_inliers);
        corridor_axis += weight * t_left;
        total_weight += weight;

        ROS_DEBUG("Left wall corridor axis: [%.3f, %.3f, %.3f], weight: %.0f",
                  t_left.x(), t_left.y(), t_left.z(), weight);
    }

    // 右墙法向量转换为走廊轴向量（在调平坐标系内）
    if (right_inliers >= config_.wall_inlier_min) {
        Eigen::Vector3d t_right = Eigen::Vector3d(0, 0, 1).cross(right_normal);
        t_right.z() = 0.0; // 只保留XY分量（在调平坐标系内）
        double norm = t_right.norm();
        if (norm > 1e-6) {
            t_right /= norm; // 归一化
        } else {
            ROS_WARN("Right wall corridor axis norm is too small: %.6f", norm);
            return prev_result_.yaw; // 返回上一帧避免跳变
        }

        // 符号对齐：确保与参考方向一致
        if (t_right.dot(ref_axis) < 0) {
            t_right = -t_right;
            ROS_DEBUG("Right wall corridor axis flipped for alignment");
        }

        // 加权融合
        double weight = static_cast<double>(right_inliers);
        corridor_axis += weight * t_right;
        total_weight += weight;

        ROS_DEBUG("Right wall corridor axis: [%.3f, %.3f, %.3f], weight: %.0f",
                  t_right.x(), t_right.y(), t_right.z(), weight);
    }

    if (total_weight > 0) {
        // 归一化得到加权平均的走廊轴向量
        corridor_axis /= total_weight;
        double norm = corridor_axis.norm();
        if (norm > 1e-6) {
            corridor_axis /= norm;
        } else {
            ROS_WARN("Weighted corridor axis norm is too small: %.6f", norm);
            return prev_result_.yaw; // 返回上一帧避免跳变
        }
    } else {
        ROS_WARN("Total weight is zero for corridor axis computation");
        return prev_result_.yaw; // 返回上一帧避免跳变
    }

    // 最终符号对齐：确保融合结果与参考方向一致
    if (corridor_axis.dot(ref_axis) < 0) {
        corridor_axis = -corridor_axis;
        ROS_DEBUG("Final corridor axis flipped for alignment");
    }

    // 缓存当前走廊轴供下一帧使用
    corridor_axis_prev_ = corridor_axis;

    ROS_DEBUG("Weighted corridor axis: [%.3f, %.3f, %.3f]", corridor_axis.x(),
              corridor_axis.y(), corridor_axis.z());

    // 用 atan2 计算航向角
    // yaw = atan2(corridor_axis.y, corridor_axis.x) - atan2(forward_axis.y,
    // forward_axis.x)
    double current_yaw_rad = std::atan2(corridor_axis.y(), corridor_axis.x());
    // 使用调平系前进轴 forward_leveled_ 作为参考
    double forward_yaw_rad =
        std::atan2(forward_leveled_.y(), forward_leveled_.x());
    double yaw_rad = current_yaw_rad - forward_yaw_rad;

    // 归一化到 [-π, π]
    while (yaw_rad > M_PI) yaw_rad -= 2 * M_PI;
    while (yaw_rad < -M_PI) yaw_rad += 2 * M_PI;

    double yaw = yaw_rad * 180.0 / M_PI;

    ROS_DEBUG("Computed yaw: %.2f degrees (current_yaw=%.2f, forward_yaw=%.2f)",
              yaw, current_yaw_rad * 180.0 / M_PI,
              forward_yaw_rad * 180.0 / M_PI);

    return yaw;
}

void HeadingEstimator::computeWallDistances(const Eigen::Vector3d& left_normal,
                                            const Eigen::Vector3d& right_normal,
                                            double left_d, double right_d,
                                            const Eigen::Matrix3d& R_level,
                                            std::vector<double>& distances)
{
    // 在调平坐标系中定义中轴线采样点
    // 注意：RANSAC 的 left_d/right_d 是基于调平坐标系点云计算的
    Eigen::Vector3d P_front_leveled =
        config_.sample_front_dist * forward_leveled_;
    Eigen::Vector3d P_back_leveled =
        config_.sample_back_dist * forward_leveled_;

    // 计算距离(在调平坐标系中)
    distances.resize(4);

    // 左墙距离: 使用点积公式 distance = |n·p + d|
    // n: 单位法向量, p: 采样点, d: 平面方程参数 (n·x + d = 0)
    if (!left_normal.isZero()) {
        distances[0] =
            std::abs(left_normal.dot(P_front_leveled) + left_d);           // LF
        distances[1] = std::abs(left_normal.dot(P_back_leveled) + left_d); // LB

        // 检查计算是否有效
        if (distances[0] > 100.0 || distances[1] > 100.0) {
            ROS_WARN("Left wall distances too large: LF=%.2f, LB=%.2f",
                     distances[0], distances[1]);
            distances[0] = -1.0;
            distances[1] = -1.0;
        }
    } else {
        distances[0] = -1.0; // Invalid
        distances[1] = -1.0;
    }

    // 右墙距离
    if (!right_normal.isZero()) {
        distances[2] =
            std::abs(right_normal.dot(P_front_leveled) + right_d); // RF
        distances[3] =
            std::abs(right_normal.dot(P_back_leveled) + right_d); // RB

        // 检查计算是否有效
        if (distances[2] > config_.wall_distance_max ||
            distances[3] > config_.wall_distance_max) {
            ROS_WARN("Right wall distances too large: RF=%.2f, RB=%.2f",
                     distances[2], distances[3]);
            distances[2] = -1.0;
            distances[3] = -1.0;
        }
    } else {
        distances[2] = -1.0; // Invalid
        distances[3] = -1.0;
    }

    ROS_DEBUG("Wall distances: LF=%.2f, LB=%.2f, RF=%.2f, RB=%.2f",
              distances[0], distances[1], distances[2], distances[3]);
}

void HeadingEstimator::applyTemporalSmoothing(double dt,
                                              EstimationResult& result)
{
    if (dt <= 0 || dt > 1.0) {
        return;
    }

    // 平滑因子: alpha = 1 - exp(-dt/tau)
    double alpha = 1.0 - std::exp(-dt / config_.temporal_smoothing_tau);

    // 平滑 roll, pitch
    result.roll = (1 - alpha) * prev_result_.roll + alpha * result.roll;
    result.pitch = (1 - alpha) * prev_result_.pitch + alpha * result.pitch;

    // === 航向角在向量域进行平滑（避免角度域的直接平均） ===
    // 将当前yaw转换为走廊轴向量
    // yaw 是相对于 forward_leveled_ 的偏角，因此：
    // corridor_axis 的角度 = forward_leveled_的角度 + yaw
    double current_yaw_rad = result.yaw * M_PI / 180.0;
    double prev_yaw_rad = prev_result_.yaw * M_PI / 180.0;

    // 使用调平系前进轴 forward_leveled_ 作为参考
    double forward_yaw_rad =
        std::atan2(forward_leveled_.y(), forward_leveled_.x());

    // 当前走廊轴向量（在调平坐标系中）
    Eigen::Vector3d current_vec(std::cos(forward_yaw_rad + current_yaw_rad),
                                std::sin(forward_yaw_rad + current_yaw_rad),
                                0.0);

    // 上一帧走廊轴向量（在调平坐标系中）
    Eigen::Vector3d prev_vec(std::cos(forward_yaw_rad + prev_yaw_rad),
                             std::sin(forward_yaw_rad + prev_yaw_rad), 0.0);

    current_vec.normalize();
    prev_vec.normalize();

    ROS_DEBUG("Yaw smoothing vectors: current=[%.3f,%.3f], prev=[%.3f,%.3f]",
              current_vec.x(), current_vec.y(), prev_vec.x(), prev_vec.y());

    // 在向量域进行指数平滑
    Eigen::Vector3d smoothed_vec = (1 - alpha) * prev_vec + alpha * current_vec;
    smoothed_vec.normalize();

    // 与上一帧对齐符号（避免180度跳变）
    if (smoothed_vec.dot(prev_vec) < 0) {
        smoothed_vec = -smoothed_vec;
        ROS_DEBUG("Flipped smoothed vector for continuity");
    }

    // 将平滑后的向量转换回角度（相对于 forward_axis）
    double smoothed_absolute_yaw_rad =
        std::atan2(smoothed_vec.y(), smoothed_vec.x());
    double smoothed_yaw_rad = smoothed_absolute_yaw_rad - forward_yaw_rad;

    // 归一化到 [-π, π]
    while (smoothed_yaw_rad > M_PI) smoothed_yaw_rad -= 2 * M_PI;
    while (smoothed_yaw_rad < -M_PI) smoothed_yaw_rad += 2 * M_PI;

    result.yaw = smoothed_yaw_rad * 180.0 / M_PI;

    ROS_DEBUG(
        "Yaw smoothing in vector domain: prev=%.2f, current=%.2f, "
        "smoothed=%.2f",
        prev_result_.yaw, result.yaw, smoothed_yaw_rad * 180.0 / M_PI);

    // 如果有效则平滑距离
    for (size_t i = 0; i < result.distances.size(); ++i) {
        if (result.distances[i] > 0 && prev_result_.distances[i] > 0) {
            result.distances[i] = (1 - alpha) * prev_result_.distances[i] +
                                  alpha * result.distances[i];
        }
    }
}

bool HeadingEstimator::checkQuality(const EstimationResult& result)
{
    if (!result.ground_valid) {
        return false;
    }

    if (result.ground_rms > config_.ground_max_rms) {
        ROS_DEBUG("Ground RMS too high: %.3fm", result.ground_rms);
        return false;
    }

    if (result.left_wall_valid && result.left_wall_rms > config_.wall_max_rms) {
        ROS_DEBUG("Left wall RMS too high: %.3fm", result.left_wall_rms);
        return false;
    }

    if (result.right_wall_valid &&
        result.right_wall_rms > config_.wall_max_rms) {
        ROS_DEBUG("Right wall RMS too high: %.3fm", result.right_wall_rms);
        return false;
    }

    if (!result.left_wall_valid && !result.right_wall_valid) {
        return false;
    }

    // 检查姿态角是否合理（掘进机设备不应有大角度倾斜）
    // 注意：欧拉角有周期性，需要考虑角度等效性
    const double max_tilt_angle = 30.0; // 最大允许倾斜角（度）

    // 对于 roll 和 pitch，检查是否超过 ±30° 或接近 ±180°（等效的反向）
    auto is_invalid_angle = [max_tilt_angle](double angle) {
        // 归一化到 [-180, 180]
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;

        // 检查是否超过 ±30° 或接近 ±180°（反向）
        return (std::abs(angle) > max_tilt_angle &&
                std::abs(angle) < 180.0 - max_tilt_angle);
    };

    if (is_invalid_angle(result.roll)) {
        ROS_WARN_THROTTLE(2.0, "Roll角异常: %.2f°（归一化后），拒绝该帧结果",
                          result.roll);
        return false;
    }

    if (is_invalid_angle(result.pitch)) {
        ROS_WARN_THROTTLE(2.0, "Pitch角异常: %.2f°（归一化后），拒绝该帧结果",
                          result.pitch);
        return false;
    }

    return true;
}

} // namespace heading_estimation
