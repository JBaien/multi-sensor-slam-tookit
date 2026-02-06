#ifndef HEADING_ESTIMATOR_H
#define HEADING_ESTIMATOR_H

#include "heading_estimation/Config.h"
#include "heading_estimation/PointCloudProcessing.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

namespace heading_estimation {

/**
 * @brief 包含估计数据的结构体
 */
struct EstimationResult {
    // 姿态(ZYX欧拉角, 弧度)
    double roll;
    double pitch;
    double yaw;

    // 四面墙距(LF, LB, RF, RB), 单位: 米
    std::vector<double> distances;  // 大小为4

    // 质量指标
    bool ground_valid;
    bool left_wall_valid;
    bool right_wall_valid;
    double ground_rms;
    double left_wall_rms;
    double right_wall_rms;
    int ground_inliers;
    int left_wall_inliers;
    int right_wall_inliers;

    // 置信度标志
    bool confidence_high;
    bool confidence_medium;
    bool confidence_low;

    EstimationResult();
    void reset();
};

/**
 * @brief 航向和墙距估计的主类
 */
class HeadingEstimator {
public:
    explicit HeadingEstimator(const HeadingConfig& config);
    ~HeadingEstimator();

    /**
     * @brief 处理新的点云帧
     * @param cloud 输入点云
     * @param timestamp 帧时间戳
     * @return 估计结果
     */
    EstimationResult processPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const ros::Time& timestamp);

    /**
     * @brief 获取前进轴向量
     */
    Eigen::Vector3d getForwardVector() const { return config_.forward_vector; }

    /**
     * @brief 获取左轴向量
     */
    Eigen::Vector3d getLeftVector() const { return left_vector_; }

private:
    /**
     * @brief 滤波和降采样点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief 提取地面平面并估计 roll/pitch
     */
    bool estimateGroundAttitude(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        Eigen::Vector3d& ground_normal,
        double& rms_error,
        int& inlier_count);

    /**
     * @brief 从地面法向量计算调平旋转
     */
    Eigen::Matrix3d computeLevelRotation(
        const Eigen::Vector3d& ground_normal);

    /**
     * @brief 提取墙面平面并估计航向角
     */
    bool estimateWallYaw(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Matrix3d& R_level,
        Eigen::Vector3d& left_wall_normal,
        Eigen::Vector3d& right_wall_normal,
        double& left_d,
        double& right_d,
        double& left_rms,
        double& right_rms,
        int& left_inliers,
        int& right_inliers);

    /**
     * @brief 判断墙面是左墙还是右墙
     */
    bool isLeftWall(const Eigen::Vector3d& wall_normal,
                    double wall_d,
                    const Eigen::Matrix3d& R_level);

    /**
     * @brief 从墙面法向量计算航向角
     */
    double computeYawFromWalls(
        const Eigen::Vector3d& left_normal,
        const Eigen::Vector3d& right_normal,
        int left_inliers,
        int right_inliers);

    /**
     * @brief 计算采样点处的墙距
     */
    void computeWallDistances(
        const Eigen::Vector3d& left_normal,
        const Eigen::Vector3d& right_normal,
        double left_d,
        double right_d,
        const Eigen::Matrix3d& R_level,
        std::vector<double>& distances);

    /**
     * @brief 应用时域平滑
     */
    void applyTemporalSmoothing(double dt, EstimationResult& result);

    /**
     * @brief 质量检查和降级处理
     */
    bool checkQuality(const EstimationResult& result);

    HeadingConfig config_;
    Eigen::Vector3d left_vector_;

    // 用于平滑的上一次结果
    EstimationResult prev_result_;
    ros::Time prev_timestamp_;
    bool first_frame_;

    std::mutex mutex_;
};

}  // namespace heading_estimation

#endif  // HEADING_ESTIMATOR_H
