#include "heading_estimation/HeadingEstimator.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace heading_estimation {

// ============================================================================
// EstimationResult 实现
// ============================================================================

EstimationResult::EstimationResult()
      : roll(0.0), pitch(0.0), yaw(0.0),
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
        confidence_low(true) {
}

void EstimationResult::reset() {
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
      : config_(config),
        first_frame_(true) {
    // 使用右手定则计算左轴: left = Z × forward
    // 右手坐标系: 右手, 大拇指指向X轴正方向、食指执行Y轴正方向、中指自然指向Z轴正方向
    // 若有两个向量A B, 则他们的叉积A × B的方向遵循右手定则
    // a 叉乘 b  a.cross(b)
    Eigen::Vector3d Z_axis(0.0, 0.0, 1.0);
    left_vector_ = Z_axis.cross(config_.forward_vector);
    left_vector_.normalize();

    ROS_INFO("Left axis vector: [%.2f, %.2f, %.2f]",
             left_vector_.x(), left_vector_.y(), left_vector_.z());
}

HeadingEstimator::~HeadingEstimator() {
}

EstimationResult HeadingEstimator::processPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const ros::Time& timestamp) {

    std::lock_guard<std::mutex> lock(mutex_);

    EstimationResult result;
    result.reset();

    if (cloud->empty()) {
        ROS_WARN_THROTTLE(2.0, "Empty point cloud received");
        return prev_result_;
    }

    try {
        // 步骤 1: 滤波和降采样
        auto filtered_cloud = filterPointCloud(cloud);
        if (filtered_cloud->empty()) {
            ROS_WARN_THROTTLE(2.0, "Point cloud empty after filtering");
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

        if (!ground_valid) {
            ROS_WARN_THROTTLE(1.0, "Ground plane estimation failed");
            if (!first_frame_) {
                result = prev_result_;
                result.confidence_low = true;
                return result;
            }
        }

        // 步骤 3：根据地面法向量计算调平旋转
        // 约定：computeLevelRotation() 返回的是 R_ws（设备坐标 → 世界坐标）
        // 即：R_ws * ground_normal = 世界坐标系的竖直方向 z_w
        Eigen::Matrix3d R_ws = computeLevelRotation(ground_normal);

        // 设备姿态矩阵：R_sw (世界坐标 → 设备坐标), 求逆
        // 设备在世界中的真实姿态应使用 R_sw，而不是调平用的 R_ws
        Eigen::Matrix3d R_sw = R_ws.transpose();

        // 从旋转矩阵中提取欧拉角（ZYX 顺序）
        // 数学含义：R = Rz(yaw) * Ry(pitch) * Rx(roll)
        // 返回向量顺序为：
        // ypr[0] -> yaw   （绕 Z 轴）
        // ypr[1] -> pitch （绕 Y 轴）
        // ypr[2] -> roll  （绕 X 轴）
        Eigen::Vector3d ypr = R_sw.eulerAngles(2, 1, 0);

        // 单激光雷达场景下 yaw 不可观测，这里只输出俯仰角和横滚角
        result.pitch = ypr[1] * 180.0 / M_PI;  // 俯仰角 Pitch（绕 Y 轴，单位：度）
        result.roll  = ypr[2] * 180.0 / M_PI;  // 横滚角 Roll（绕 X 轴，单位：度）


        // 步骤 4: 估计墙面和航向角
        Eigen::Vector3d left_wall_normal, right_wall_normal;
        double left_d = 0.0, right_d = 0.0;
        double left_rms = 0.0, right_rms = 0.0;
        int left_inliers = 0, right_inliers = 0;

        bool wall_valid = estimateWallYaw(
            filtered_cloud, R_sw,
            left_wall_normal, right_wall_normal,
            left_d, right_d,
            left_rms, right_rms,
            left_inliers, right_inliers);

        result.left_wall_valid = (left_inliers >= config_.wall_inlier_min);
        result.right_wall_valid = (right_inliers >= config_.wall_inlier_min);
        result.left_wall_rms = left_rms;
        result.right_wall_rms = right_rms;
        result.left_wall_inliers = left_inliers;
        result.right_wall_inliers = right_inliers;

        if (wall_valid) {
            // Compute yaw from wall normals
            result.yaw = computeYawFromWalls(
                left_wall_normal, right_wall_normal,
                left_inliers, right_inliers);

            // 步骤 5: 计算四面墙距
            computeWallDistances(left_wall_normal, right_wall_normal,
                                left_d, right_d, R_sw, result.distances);
        } else {
            ROS_WARN_THROTTLE(1.0, "Wall plane estimation failed");
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

        // 步骤 7: 质量检查
        result.confidence_high = checkQuality(result);
        result.confidence_low = !result.ground_valid ||
                                (!result.left_wall_valid && !result.right_wall_valid);
        result.confidence_medium = !result.confidence_high && !result.confidence_low;

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
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 体素网格降采样
    filtered = PointCloudProcessing::voxelFilter(cloud, config_.voxel_size);

    // 离群点移除
    filtered = PointCloudProcessing::outlierRemoval(
        filtered, config_.outlier_mean_k, config_.outlier_std_mul);

    return filtered;
}

bool HeadingEstimator::estimateGroundAttitude(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        Eigen::Vector3d& ground_normal,
        double& rms_error,
        int& inlier_count) {

    // 地面估计 ROI 滤波
    auto roi_cloud = PointCloudProcessing::roiFilter(
        cloud, config_.ground_roi_radius);

    if (roi_cloud->size() < config_.ground_inlier_min) {
        ROS_DEBUG("ROI Points < inlier_min: %lu", roi_cloud->size());
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
    std::nth_element(z_values.begin(),
                        z_values.begin() + static_cast<size_t>(z_values.size() * config_.ground_z_percentile),
                        z_values.end());
    float z_threshold = z_values[static_cast<size_t>(z_values.size() * config_.ground_z_percentile)];

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
        return false;
    }

    // RANSAC 平面拟合
    double d;
    // 最大迭代次数: 200 内点距离: 0.05
    bool success = PointCloudProcessing::ransacPlane(
        ground_cloud, ground_normal, d, rms_error, inlier_count,
        200, 0.05);

    if (!success || inlier_count < config_.ground_inlier_min) {
        ROS_DEBUG("Ground plane RANSAC failed: inliers=%d", inlier_count);
        return false;
    }

    // 使用加权最小二乘优化
    auto inliers = PointCloudProcessing::extractPlaneInliers(
        ground_cloud, ground_normal, d, 0.05);
    auto weights = PointCloudProcessing::computeTukeyWeights(
        inliers, ground_normal, d);
    PointCloudProcessing::refinePlaneWeighted(
        inliers, ground_normal, d, weights);

    // 重新计算 RMS
    rms_error = PointCloudProcessing::computePlaneRMS(inliers, ground_normal, d);

    // 检查地面是否近似水平
    double z_component = std::abs(ground_normal.z());
    if (z_component < std::cos(config_.ground_normal_threshold * M_PI / 180.0)) {
        ROS_DEBUG("Ground plane not horizontal enough: |n_z|=%.3f", z_component);
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
        const Eigen::Matrix3d& R_level,
        Eigen::Vector3d& left_wall_normal,
        Eigen::Vector3d& right_wall_normal,
        double& left_d,
        double& right_d,
        double& left_rms,
        double& right_rms,
        int& left_inliers,
        int& right_inliers) {

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
    auto leveled_cloud = PointCloudProcessing::transformPointCloud(cloud, R_level);

    // 墙面高度滤波
    auto height_filtered = PointCloudProcessing::passThroughZ(
        leveled_cloud, config_.wall_z_min, config_.wall_z_max);

    if (height_filtered->size() < config_.wall_inlier_min) {
        ROS_DEBUG("Insufficient wall points: %lu", height_filtered->size());
        return false;
    }

    // 顺序提取平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining = height_filtered;
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> d_values;
    std::vector<double> rms_values;
    std::vector<int> inlier_counts;

    for (int i = 0; i < 2; ++i) {
        if (remaining->size() < config_.wall_inlier_min) {
            break;
        }

        Eigen::Vector3d normal;
        double d, rms;
        int inliers;
        bool success = PointCloudProcessing::ransacPlane(
            remaining, normal, d, rms, inliers,
            300, 0.05);

        if (!success || inliers < config_.wall_inlier_min) {
            break;
        }

        // 检查是否垂直
        if (!PointCloudProcessing::isVerticalPlane(normal, config_.wall_vertical_threshold)) {
            ROS_DEBUG("Plane %d not vertical: n_z=%.3f", i, normal.z());
            break;
        }

        normals.push_back(normal);
        d_values.push_back(d);
        rms_values.push_back(rms);
        inlier_counts.push_back(inliers);

        // 为下次迭代提取内点和外点
        auto inlier_cloud = PointCloudProcessing::extractPlaneInliers(
            remaining, normal, d, 0.05);
        remaining = PointCloudProcessing::extractPlaneOutliers(
            remaining, normal, d, 0.05);
    }

    if (normals.empty()) {
        ROS_DEBUG("No valid vertical planes found");
        return false;
    }

    // 分类墙面为左/右墙
    for (size_t i = 0; i < normals.size(); ++i) {
        if (isLeftWall(normals[i], d_values[i], R_level)) {
            left_wall_normal = normals[i];
            left_d = d_values[i];
            left_rms = rms_values[i];
            left_inliers = inlier_counts[i];
        } else {
            right_wall_normal = normals[i];
            right_d = d_values[i];
            right_rms = rms_values[i];
            right_inliers = inlier_counts[i];
        }
    }

    // 检查两面墙是否存在且平行
    if (left_inliers > 0 && right_inliers > 0) {
        double parallel_dot = std::abs(left_wall_normal.dot(right_wall_normal));
        if (parallel_dot < config_.wall_parallel_cos) {
            ROS_DEBUG("Walls not parallel: dot=%.3f", parallel_dot);
            return false;
        }
    }

    return (left_inliers > 0 || right_inliers > 0);
}

bool HeadingEstimator::isLeftWall(const Eigen::Vector3d& wall_normal,
                                    double wall_d,
                                    const Eigen::Matrix3d& R_level) {
    // 计算平面上到原点最近的点: p0 = -d * n
    Eigen::Vector3d p0 = -wall_d * wall_normal;

    // 变换 left_vector 到调平坐标系
    Eigen::Vector3d left_leveled = R_level.transpose() * left_vector_;

    // Check which side the closest point is on
    double projection = p0.dot(left_leveled);

    return projection > 0;
}

double HeadingEstimator::computeYawFromWalls(
        const Eigen::Vector3d& left_normal,
        const Eigen::Vector3d& right_normal,
        int left_inliers,
        int right_inliers) {

    Eigen::Vector3d heading_vector(0, 0, 0);
    double total_weight = 0.0;

    // 左墙贡献
    if (left_inliers >= config_.wall_inlier_min) {
        Eigen::Vector3d t_left = Eigen::Vector3d(0, 0, 1).cross(left_normal);
        t_left.normalize();
        double weight = static_cast<double>(left_inliers);
        heading_vector += weight * t_left;
        total_weight += weight;
    }

    // 右墙贡献
    if (right_inliers >= config_.wall_inlier_min) {
        Eigen::Vector3d t_right = Eigen::Vector3d(0, 0, 1).cross(right_normal);
        t_right.normalize();
        double weight = static_cast<double>(right_inliers);
        heading_vector += weight * t_right;
        total_weight += weight;
    }

    if (total_weight > 0) {
        heading_vector /= total_weight;
        heading_vector.normalize();
    }

    // 计算航向角为 forward_axis 到 heading_vector 的夹角(绕 Z 轴)
    double yaw = std::atan2(
        heading_vector.cross(config_.forward_vector).z(),
        heading_vector.dot(config_.forward_vector));

    // 转换为角度
    yaw = yaw * 180.0 / M_PI;

    return yaw;
}

void HeadingEstimator::computeWallDistances(
        const Eigen::Vector3d& left_normal,
        const Eigen::Vector3d& right_normal,
        double left_d,
        double right_d,
        const Eigen::Matrix3d& R_level,
        std::vector<double>& distances) {

    // 在基坐标系中定义中轴线采样点
    Eigen::Vector3d P_front = config_.sample_front_dist * config_.forward_vector;
    Eigen::Vector3d P_back = config_.sample_back_dist * config_.forward_vector;

    // 变换到调平坐标系
    Eigen::Vector3d P_front_leveled = R_level.transpose() * P_front;
    Eigen::Vector3d P_back_leveled = R_level.transpose() * P_back;

    // 计算距离(在调平坐标系中)
    distances.resize(4);

    // 左墙距离
    if (!left_normal.isZero()) {
        distances[0] = std::abs(left_normal.dot(P_front_leveled) + left_d);  // LF
        distances[1] = std::abs(left_normal.dot(P_back_leveled) + left_d);   // LB
    } else {
        distances[0] = -1.0;  // Invalid
        distances[1] = -1.0;
    }

    // 右墙距离
    if (!right_normal.isZero()) {
        distances[2] = std::abs(right_normal.dot(P_front_leveled) + right_d);  // RF
        distances[3] = std::abs(right_normal.dot(P_back_leveled) + right_d);   // RB
    } else {
        distances[2] = -1.0;  // Invalid
        distances[3] = -1.0;
    }
}

void HeadingEstimator::applyTemporalSmoothing(double dt, EstimationResult& result) {
    if (dt <= 0 || dt > 1.0) {
        return;
    }

    // 平滑因子: alpha = 1 - exp(-dt/tau)
    double alpha = 1.0 - std::exp(-dt / config_.temporal_smoothing_tau);

    // 平滑 roll, pitch, yaw
    result.roll = (1 - alpha) * prev_result_.roll + alpha * result.roll;
    result.pitch = (1 - alpha) * prev_result_.pitch + alpha * result.pitch;

    // 航向角速率限制(yaw 现在是角度)
    double yaw_diff = result.yaw - prev_result_.yaw;
    while (yaw_diff > 180.0) yaw_diff -= 360.0;
    while (yaw_diff < -180.0) yaw_diff += 360.0;

    double max_yaw_rate_deg = config_.max_yaw_rate * 180.0 / M_PI;  // Convert rad/frame to deg/frame
    if (std::abs(yaw_diff) > max_yaw_rate_deg) {
        yaw_diff = (yaw_diff > 0 ? 1 : -1) * max_yaw_rate_deg;
    }

    result.yaw = prev_result_.yaw + yaw_diff;
    while (result.yaw > 180.0) result.yaw -= 360.0;
    while (result.yaw < -180.0) result.yaw += 360.0;

    // 如果有效则平滑距离
    for (size_t i = 0; i < result.distances.size(); ++i) {
        if (result.distances[i] > 0 && prev_result_.distances[i] > 0) {
            result.distances[i] = (1 - alpha) * prev_result_.distances[i] +
                                alpha * result.distances[i];
        }
    }
}

bool HeadingEstimator::checkQuality(const EstimationResult& result) {
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

    if (result.right_wall_valid && result.right_wall_rms > config_.wall_max_rms) {
        ROS_DEBUG("Right wall RMS too high: %.3fm", result.right_wall_rms);
        return false;
    }

    if (!result.left_wall_valid && !result.right_wall_valid) {
        return false;
    }

    return true;
}

}  // namespace heading_estimation
