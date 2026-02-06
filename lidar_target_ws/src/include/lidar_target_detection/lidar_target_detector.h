#ifndef LIDAR_TARGET_DETECTOR_H
#define LIDAR_TARGET_DETECTOR_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>
// 自定义点类型，包含XYZIRT信息
struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (float, time, time)
)
class LidarTargetDetector
{
public:
    LidarTargetDetector(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~LidarTargetDetector() {}
private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher target_pub_;
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher clusters_pub_;
    
    // 参数
    double intensity_threshold_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double min_arc_angle_;
    double max_arc_angle_;
    double max_rmse_threshold_;
    double min_target_radius_;
    double max_target_radius_;
    
    // 卡尔曼滤波器状态
    Eigen::Vector4d state_; // [x, y, vx, vy]
    Eigen::Matrix4d P_;     // 状态协方差矩阵
    Eigen::Matrix<double, 2, 4> H_; // 观测矩阵
    Eigen::Matrix4d Q_;     // 过程噪声
    Eigen::Matrix2d R_;     // 观测噪声
    
    // 回调函数
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    
    // 处理函数
    pcl::PointCloud<PointXYZIRT>::Ptr filterByIntensity(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
    std::vector<pcl::PointCloud<PointXYZIRT>::Ptr> extractClusters(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
    bool fitArcAndGetCenter(const pcl::PointCloud<PointXYZIRT>::Ptr& cluster, 
                           geometry_msgs::PointStamped& center);
    void updateKalmanFilter(const geometry_msgs::PointStamped& measurement);
    void publishClustersVisualization(const std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>& clusters, 
                                      const std_msgs::Header& header);
};
#endif // LIDAR_TARGET_DETECTOR_H