#include "heading_estimation/PointCloudProcessing.h"
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/transforms.h>
#include <cmath>

namespace heading_estimation {

// 体素网格降采样
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::voxelFilter(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      double voxel_size) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 点云滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    // 是否保留体素布局
    // voxel_filter.setSaveLeafLayout(true);
    // 体素内最少点数, 抑制噪点体素
    voxel_filter.setMinimumPointsNumberPerVoxel(2);
    voxel_filter.filter(*filtered);

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::outlierRemoval(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      int mean_k,
      double std_mul) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 统计离群点滤波器SOR
    /*
      TODO:
      1. 输入点云中的NAN/inf 或明显越界点的剔除
      2. 粉尘浓度大时加入点云强度进行剔除粉尘引入的干扰数据
    */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    // mean_k 每个点参与统计的最近临近点数量
    sor.setMeanK(mean_k);
    // std_mul 标准差倍数阈值
    sor.setStddevMulThresh(std_mul);
    sor.filter(*filtered);

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::passThroughZ(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        double z_min,
        double z_max,
        bool negative) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.setFilterLimitsNegative(negative);
    pass.filter(*filtered);

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::roiFilter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        double radius) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filtered->reserve(cloud->size());

    double radius_squared = radius * radius;

    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            // 圆柱体ROI：只限制水平距离(x,y)，不限制z
            double dist_xy_sq = pt.x * pt.x + pt.y * pt.y;
            if (dist_xy_sq <= radius_squared) {
                filtered->points.push_back(pt);
            }
        }
    }

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::transformPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Matrix3d& rotation) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    transformed->resize(cloud->size());

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation.cast<float>();

    pcl::transformPointCloud(*cloud, *transformed, transform);

    return transformed;
}

bool PointCloudProcessing::ransacPlane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        Eigen::Vector3d& normal,
        double& d,
        double& rms_error,
        int& inlier_count,
        int max_iterations,
        double distance_threshold) {

    if (cloud->size() < 3) {
        return false;
    }

    // 创建平面模型
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    
    // 创建Ransac求解器
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(distance_threshold);
    // 最大迭代次数
    ransac.setMaxIterations(max_iterations);
    ransac.computeModel();
    
    // inliners中存的是索引
    std::vector<int> inliers;
    ransac.getInliers(inliers);

    // TODO: 内点 >> 100+
    if (inliers.size() < 20) {
        ROS_WARN_THROTTLE(1.0, "拟合地面平面点数量过少!");
        return false;
    }

    Eigen::VectorXf model_coefficients;
    ransac.getModelCoefficients(model_coefficients);

    // ax+by+cz+d = 0
    normal = Eigen::Vector3d(model_coefficients[0],
                            model_coefficients[1],
                            model_coefficients[2]);
    d = model_coefficients[3];

    // 归一化
    // 欧几里得范数: 向量的长度, 大小反映"尺度"
    double norm = normal.norm();
    if (norm < 1e-6) {
        return false;
    }
    normal /= norm;
    d /= norm;

    // 计算 RMS 误差
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    inlier_cloud->points.reserve(inliers.size());
    
    // 索引到实体点的映射
    for (int idx : inliers) {
        inlier_cloud->points.push_back(cloud->points[idx]);
    }

    rms_error = computePlaneRMS(inlier_cloud, normal, d);
    inlier_count = static_cast<int>(inliers.size());

    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::extractPlaneInliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Vector3d& normal,
        double d,
        double threshold) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>);
    inliers->reserve(cloud->size());

    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            double distance = std::abs(normal.x() * pt.x +
                                    normal.y() * pt.y +
                                    normal.z() * pt.z + d);
            if (distance <= threshold) {
                inliers->points.push_back(pt);
            }
        }
    }

    return inliers;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::extractPlaneOutliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Vector3d& normal,
        double d,
        double threshold) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>);
    outliers->reserve(cloud->size());

    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            double distance = std::abs(normal.x() * pt.x +
                                    normal.y() * pt.y +
                                    normal.z() * pt.z + d);
            if (distance > threshold) {
                outliers->points.push_back(pt);
            }
        }
    }

    return outliers;
}

double PointCloudProcessing::computePlaneRMS(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Vector3d& normal,
        double d) {

    if (cloud->empty()) {
        return 0.0;
    }

    double sum_sq = 0.0;
    size_t count = 0;

    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            double distance = normal.x() * pt.x +
                            normal.y() * pt.y +
                            normal.z() * pt.z + d;
            sum_sq += distance * distance;
            ++count;
        }
    }

    return (count > 0) ? std::sqrt(sum_sq / count) : 0.0;
}

bool PointCloudProcessing::refinePlaneWeighted(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        Eigen::Vector3d& normal,
        double& d,
        const std::vector<double>& weights) {

    if (cloud->size() < 3 || weights.size() != cloud->size()) {
        return false;
    }

    // 加权最小二乘: 最小化 sum(w_i * (n·p_i + d)^2)
    // 约束 ||n|| = 1

    // 计算加权质心
    double sum_w = 0.0;
    Eigen::Vector3d weighted_centroid(0, 0, 0);

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (std::isfinite(cloud->points[i].x) &&
                std::isfinite(cloud->points[i].y) &&
                std::isfinite(cloud->points[i].z)) {
            sum_w += weights[i];
            weighted_centroid.x() += weights[i] * cloud->points[i].x;
            weighted_centroid.y() += weights[i] * cloud->points[i].y;
            weighted_centroid.z() += weights[i] * cloud->points[i].z;
        }
    }

    if (sum_w < 1e-6) {
        return false;
    }

    weighted_centroid /= sum_w;

    // 计算加权协方差
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (std::isfinite(cloud->points[i].x) &&
                std::isfinite(cloud->points[i].y) &&
                std::isfinite(cloud->points[i].z)) {
            Eigen::Vector3d p(cloud->points[i].x,
                            cloud->points[i].y,
                            cloud->points[i].z);
            Eigen::Vector3d diff = p - weighted_centroid;
            covariance += weights[i] * diff * diff.transpose();
        }
    }

    // 特征分解: 法向量是最小特征值对应的特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);

    if (eigensolver.info() != Eigen::Success) {
        return false;
    }

    normal = eigensolver.eigenvectors().col(0);
    d = -normal.dot(weighted_centroid);

    // 确保方向一致
    if (normal.z() < 0) {
        normal = -normal;
        d = -d;
    }

    return true;
}

std::vector<double> PointCloudProcessing::computeTukeyWeights(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Vector3d& normal,
        double d,
        double tuning_constant) {

    std::vector<double> weights;
    weights.reserve(cloud->size());

    // 使用 MAD (中位绝对偏差) 估计尺度
    std::vector<double> residuals;
    residuals.reserve(cloud->size());

    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            double r = normal.x() * pt.x + normal.y() * pt.y + normal.z() * pt.z + d;
            residuals.push_back(std::abs(r));
        }
    }

    if (residuals.empty()) {
        weights.assign(cloud->size(), 1.0);
        return weights;
    }

    std::nth_element(residuals.begin(),
                        residuals.begin() + residuals.size() / 2,
                        residuals.end());
    double mad = residuals[residuals.size() / 2];
    double scale = 1.4826 * mad;  // Consistent estimator for normal distribution

    if (scale < 1e-6) {
        scale = 1e-6;
    }

    // 计算 Tukey 权重
    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            double r = std::abs(normal.x() * pt.x + normal.y() * pt.y + normal.z() * pt.z + d);
            double u = r / (tuning_constant * scale);

            if (u <= 1.0) {
                double w = std::pow(1.0 - u * u, 2);
                weights.push_back(w);
            } else {
                weights.push_back(0.0);
            }
        } else {
            weights.push_back(0.0);
        }
    }

    return weights;
}

bool PointCloudProcessing::isVerticalPlane(
        const Eigen::Vector3d& normal,
        double z_threshold) {
    return std::abs(normal.z()) < z_threshold;
}

bool PointCloudProcessing::arePlanesParallel(
        const Eigen::Vector3d& n1,
        const Eigen::Vector3d& n2,
        double cos_threshold) {
    double dot = std::abs(n1.dot(n2));
    return dot >= cos_threshold;
}

}  // namespace heading_estimation
