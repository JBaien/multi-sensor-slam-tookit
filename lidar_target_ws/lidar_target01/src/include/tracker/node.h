#ifndef LIDAR_REFLECTOR_TARGET_TRACKER_NODE_H
#define LIDAR_REFLECTOR_TARGET_TRACKER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
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

#include "tracker/config.h"
#include "tracker/kalman_filter.h"

namespace tracker
{

class TargetTrackerNode
{
public:
  explicit TargetTrackerNode(const ros::NodeHandle& nh = ros::NodeHandle(),
                            const ros::NodeHandle& pnh = ros::NodeHandle("~"));

  void init();
  void run();

private:
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg);
  void handleNoMeasurement();
  void publishOutputs(const std::string& frame_id, const ros::Time& stamp, bool measured);

  void loadConfig();
  void setupSubscribers();
  void setupPublishers();

  // TCP服务器相关
  bool initTCPServer();
  void closeTCPServer();
  void tcpServerThread();
  void heartbeatThread();
  void handleModbusRequest(int client_fd, const uint8_t* request, ssize_t length);
  void removeClient(int client_fd);
  void sendTCPData(double lidar_x, double lidar_y);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  TrackerConfig config_;

  ros::Subscriber sub_;
  ros::Publisher pub_target_;
  ros::Publisher pub_vehicle_;
  ros::Publisher pub_status_;
  ros::Publisher pub_marker_;

  ros::Time last_stamp_;
  KalmanFilterCV kf_;
  int missed_{0};
  bool have_last_meas_{false};
  double last_meas_x_{0};
  double last_meas_y_{0};

  // TCP服务器成员变量
  int tcp_server_fd_;
  std::vector<int> tcp_client_fds_;
  int tcp_port_;
  std::atomic<bool> tcp_server_running_;
  std::thread tcp_server_thread_;
  std::unordered_map<int, std::vector<uint8_t>> client_rx_caches_;
  static constexpr size_t REG_COUNT = 100;
  std::array<std::atomic<uint16_t>, REG_COUNT> modbus_registers_;
  std::atomic<uint16_t> heartbeat_;
  std::thread heartbeat_thread_;

  // 寄存器地址定义
  static const int REG_HEARTBEAT = 10;
  static const int REG_X_COORD = 11;  // int16, 单位: mm (雷达相对小车X)
  static const int REG_Y_COORD = 12;  // int16, 单位: mm (雷达相对小车Y)
  static const int REG_DISTANCE = 13; // uint16, 单位: mm
  static const int REG_ANGLE = 14;    // int16, 单位: 0.1°
};

} // namespace tracker

#endif // LIDAR_REFLECTOR_TARGET_TRACKER_NODE_H
