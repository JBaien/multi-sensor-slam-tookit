#ifndef POINT_CLOUD_PROCESSING_H
#define POINT_CLOUD_PROCESSING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

namespace heading_estimation {

/**
 * @brief 点云处理工具类
 */
class PointCloudProcessing {
public:
    /**
     * @brief 体素网格降采样
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        double voxel_size);

    /**
     * @brief 统计离群点移除
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr outlierRemoval(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int mean_k,
        double std_mul);

    /**
     * @brief 直通滤波器(Z轴)
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughZ(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        double z_min,
        double z_max,
        bool negative = false);

    /**
     * @brief 直通滤波器(按半径的ROI)
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr roiFilter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        double radius);

    /**
     * @brief 变换点云
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Matrix3d& rotation);

    /**
     * @brief RANSAC 平面拟合
     */
    static bool ransacPlane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        Eigen::Vector3d& normal,
        double& d,
        double& rms_error,
        int& inlier_count,
        int max_iterations = 200,
        double distance_threshold = 0.05);

    /**
     * @brief 提取平面内点
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr extractPlaneInliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Vector3d& normal,
        double d,
        double threshold);

    /**
     * @brief 提取平面外点
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr extractPlaneOutliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Vector3d& normal,
        double d,
        double threshold);

    /**
     * @brief 计算平面 RMS 误差
     */
    static double computePlaneRMS(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Vector3d& normal,
        double d);

    /**
     * @brief 使用加权最小二乘优化平面
     */
    static bool refinePlaneWeighted(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        Eigen::Vector3d& normal,
        double& d,
        const std::vector<double>& weights);

    /**
     * @brief 检查平面是否垂直
     */
    static bool isVerticalPlane(
        const Eigen::Vector3d& normal,
        double z_threshold);

    /**
     * @brief 检查两个平面是否平行
     */
    static bool arePlanesParallel(
        const Eigen::Vector3d& n1,
        const Eigen::Vector3d& n2,
        double cos_threshold);

    /**
     * @brief 计算鲁棒拟合的 Tukey 权重
     */
    static std::vector<double> computeTukeyWeights(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Vector3d& normal,
        double d,
        double tuning_constant = 4.685);
};

}  // namespace heading_estimation

#endif  // POINT_CLOUD_PROCESSING_H
