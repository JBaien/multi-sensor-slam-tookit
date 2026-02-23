/**
 * @file target_detector.h
 * @brief 激光雷达反光标靶检测器
 *
 * 基于点云处理和圆拟合的标靶检测算法。
 * 处理流程: 点云输入 -> 强度滤波 -> ROI过滤 -> 聚类 -> 圆拟合 -> 卡尔曼滤波 -> TCP发布
 */

#ifndef LIDAR_TARGET_LOCALIZATION_TARGET_DETECTOR_H
#define LIDAR_TARGET_LOCALIZATION_TARGET_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>

#include "lidar_target_localization/TargetPosition.h"
#include "lidar_target_localization/kalman_filter.h"

#include <atomic>
#include <array>
#include <thread>
#include <unordered_map>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// 使用带强度的点类型
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

/**
 * @class TargetDetector
 * @brief 激光雷达反光标靶检测器
 *
 * 核心功能:
 * 1. 订阅激光雷达点云数据
 * 2. 通过强度阈值提取反光点
 * 3. 欧式聚类分离目标
 * 4. 最小二乘法圆拟合确定圆心位置
 * 5. 卡尔曼滤波平滑输出
 * 6. TCP Modbus服务器发布坐标
 */
class TargetDetector
{
public:
    TargetDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    // ==================== ROS相关 ====================
    ros::NodeHandle nh_, pnh_;                           // ROS节点句柄
    ros::Subscriber lidar_sub_;                          // 点云订阅者
    ros::Publisher  target_pub_;                         // 目标位置发布者
    ros::Publisher  filtered_cloud_pub_;                 // 调试用：发布强度滤波后的点云

    // ==================== 配置参数 ====================
    std::string lidar_topic_;                            // 输入点云话题
    std::string output_topic_;                           // 输出目标位置话题
    double intensity_threshold_;                         // 强度阈值（反光点过滤）
    double min_range_, max_range_;                       // 距离范围 [m]
    double min_height_, max_height_;                    // 高度范围 [m]
    double cluster_tolerance_;                           // 聚类距离容差 [m]
    int    min_cluster_size_, max_cluster_size_;        // 聚类点数范围
    double min_radius_, max_radius_;                     // 标靶半径范围 [m]
    bool   use_kalman_;                                  // 是否使用卡尔曼滤波
    double process_noise_q_;                             // 卡尔曼过程噪声
    double measurement_noise_r_;                         // 卡尔曼观测噪声
    bool   use_tracking_;                                // 是否使用跟踪
    double tracking_search_radius_;                      // 跟踪搜索半径 [m]
    int    max_lost_frames_;                             // 最大丢失帧数

    // ==================== 内部状态 ====================
    KalmanFilter2D kf_;                                 // 卡尔曼滤波器
    bool   tracking_initialized_;                       // 跟踪初始化标志
    double last_target_x_, last_target_y_;              // 上一帧目标位置
    int    lost_frame_count_;                            // 丢失帧计数
    ros::Time last_stamp_;                               // 上一帧时间戳

    // ==================== TCP服务器 ====================
    int tcp_server_fd_;                                  // TCP服务器socket
    std::vector<int> tcp_client_fds_;                    // TCP客户端列表
    int tcp_port_;                                       // TCP端口
    std::atomic<bool> tcp_server_running_;               // TCP服务器运行标志
    std::thread tcp_server_thread_;                      // TCP服务器线程
    std::unordered_map<int, std::vector<uint8_t>> client_rx_caches_;  // 客户端接收缓存
    static constexpr size_t REG_COUNT = 100;             // Modbus寄存器数量
    std::array<std::atomic<uint16_t>, REG_COUNT> modbus_registers_;   // Modbus寄存器
    std::atomic<uint16_t> heartbeat_;                    // 心跳计数器
    std::thread heartbeat_thread_;                       // 心跳线程

    // Modbus寄存器地址定义
    static const int REG_HEARTBEAT = 10;                // 心跳寄存器
    static const int REG_X_COORD = 11;                   // X坐标 (int16, 单位: mm, 雷达相对小车)
    static const int REG_Y_COORD = 12;                   // Y坐标 (int16, 单位: mm, 雷达相对小车)
    static const int REG_DISTANCE = 13;                 // 距离 (uint16, 单位: mm, 固定1000)
    static const int REG_ANGLE = 14;                     // 角度 (int16, 单位: 0.1°, 固定0)

    // ==================== TCP服务器方法 ====================
    bool initTCPServer();                               // 初始化TCP服务器
    void closeTCPServer();                               // 关闭TCP服务器
    void tcpServerThread();                             // TCP服务器线程
    void heartbeatThread();                             // 心跳线程
    void handleModbusRequest(int client_fd, const uint8_t* request, ssize_t length);  // 处理Modbus请求
    void removeClient(int client_fd);                    // 移除客户端
    void sendTCPData(double lidar_x, double lidar_y);    // 发布雷达相对于小车的坐标

    // ==================== 核心回调 ====================
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);  // 点云回调

    // ==================== 处理流程 ====================
    PointCloud::Ptr intensityFilter(const PointCloud::Ptr& cloud);       // 强度滤波
    PointCloud::Ptr rangeHeightFilter(const PointCloud::Ptr& cloud);    // 距离高度滤波
    PointCloud::Ptr trackingROIFilter(const PointCloud::Ptr& cloud);     // 跟踪ROI滤波

    std::vector<pcl::PointIndices> euclideanClustering(const PointCloud::Ptr& cloud);  // 欧式聚类

    // 圆拟合：最小二乘法拟合2D圆心，返回是否成功
    bool fitCircle(const PointCloud::Ptr& cloud,
                   const pcl::PointIndices& indices,
                   double& cx, double& cy, double& radius);

    void loadParams();                                   // 加载参数
};

#endif // LIDAR_TARGET_LOCALIZATION_TARGET_DETECTOR_H
