#ifndef LIDAR_TARGET_DETECTOR_H
#define LIDAR_TARGET_DETECTOR_H

#include <arpa/inet.h>
#include <geometry_msgs/PointStamped.h>
#include <netinet/in.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

// 自定义点类型，包含XYZIRT信息
struct PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring,
                                                       ring)(float, time, time))

class LidarTargetDetector {
   public:
    LidarTargetDetector(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~LidarTargetDetector()
    {
        closeTCPServer();
        tcp_server_running_ = false;
        if (heartbeat_thread_.joinable()) {
            heartbeat_thread_.join();
        }
    }

   private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher target_pub_;
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher clusters_pub_;

    // TCP服务器相关
    int tcp_server_fd_;
    int tcp_client_fd_;
    int tcp_port_;
    std::atomic<bool> tcp_server_running_;
    std::thread tcp_server_thread_;
    std::mutex tcp_mutex_;

    // Modbus相关
    uint16_t modbus_registers_[100];  // Modbus寄存器数组
    std::atomic<uint16_t> heartbeat_; // 心跳计数器
    std::thread heartbeat_thread_;    // 心跳线程

    // 寄存器地址定义
    static const int REG_HEARTBEAT = 10;
    static const int REG_X_COORD = 11;  // int16, 单位: mm
    static const int REG_Y_COORD = 12;  // int16, 单位: mm
    static const int REG_DISTANCE = 13; // uint16, 移动距离, 单位: mm
    static const int REG_ANGLE = 14;    // int16, 移动角度, 单位: 0.1°

    // TCP服务器方法
    bool initTCPServer();
    void closeTCPServer();
    void tcpServerThread();
    void sendTCPData(const geometry_msgs::PointStamped& position);
    void heartbeatThread();
    void handleModbusRequest(uint8_t* request, ssize_t length);

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
    Eigen::Vector4d state_;         // [x, y, vx, vy]
    Eigen::Matrix4d P_;             // 状态协方差矩阵
    Eigen::Matrix<double, 2, 4> H_; // 观测矩阵
    Eigen::Matrix4d Q_;             // 过程噪声
    Eigen::Matrix2d R_;             // 观测噪声

    // 回调函数
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    // 处理函数
    pcl::PointCloud<PointXYZIRT>::Ptr filterByIntensity(
        const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
    std::vector<pcl::PointCloud<PointXYZIRT>::Ptr> extractClusters(
        const pcl::PointCloud<PointXYZIRT>::Ptr& cloud);
    bool fitArcAndGetCenter(const pcl::PointCloud<PointXYZIRT>::Ptr& cluster,
                            geometry_msgs::PointStamped& center);
    void updateKalmanFilter(const geometry_msgs::PointStamped& measurement);
    void publishClustersVisualization(
        const std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>& clusters,
        const std_msgs::Header& header);
};

#endif // LIDAR_TARGET_DETECTOR_H
