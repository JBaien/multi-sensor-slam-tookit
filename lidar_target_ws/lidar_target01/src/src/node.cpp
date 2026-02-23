#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cmath>
#include <chrono>
#include <sys/select.h>
#include <netinet/tcp.h>
#include "tracker/node.h"
#include "tracker/circle_fit.h"

namespace {

static bool setNonBlocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return false;
    return true;
}

static void setSockBuf(int fd, int snd_bytes, int rcv_bytes)
{
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &snd_bytes, sizeof(snd_bytes));
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcv_bytes, sizeof(rcv_bytes));
}

static void setNoDelay(int fd)
{
    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
}

static void setKeepAlive(int fd)
{
    int keepalive = 1;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));

    int keepidle = 30;
    int keepintvl = 5;
    int keepcnt = 3;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));
}

static bool sendAllWithDeadlineMs(int fd, const uint8_t* data, size_t len,
                                  int deadline_ms)
{
    using clock = std::chrono::steady_clock;
    auto start = clock::now();

    size_t sent_total = 0;
    while (sent_total < len) {
        ssize_t n = send(fd, data + sent_total, len - sent_total, MSG_NOSIGNAL);
        if (n > 0) {
            sent_total += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            auto now = clock::now();
            int elapsed = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                      start)
                    .count());
            int remain = deadline_ms - elapsed;
            if (remain <= 0) return false;

            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);

            struct timeval tv;
            tv.tv_sec = remain / 1000;
            tv.tv_usec = (remain % 1000) * 1000;

            int r = select(fd + 1, nullptr, &wfds, nullptr, &tv);
            if (r > 0) continue;
            if (r < 0 && errno == EINTR) continue;
            return false;
        }
        return false;
    }
    return true;
}

} // namespace

namespace tracker
{

TargetTrackerNode::TargetTrackerNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh),
    tcp_server_fd_(-1),
    tcp_port_(5050),
    tcp_server_running_(false),
    heartbeat_(0)
{
  loadConfig();
  setupSubscribers();
  setupPublishers();

  ROS_INFO("lidar_reflector_target_tracker started. sub=%s intensity_th=%.1f",
           config_.input_topic.c_str(), config_.intensity_threshold);
}

void TargetTrackerNode::init()
{
}

void TargetTrackerNode::run()
{
  ros::spin();
}

void TargetTrackerNode::loadConfig()
{
  pnh_.param<std::string>("input_topic", config_.input_topic, std::string("/points_raw"));
  pnh_.param<double>("intensity_threshold", config_.intensity_threshold, 150.0);

  pnh_.param<bool>("use_z_filter", config_.use_z_filter, true);
  pnh_.param<double>("z_min", config_.z_min, -1.0);
  pnh_.param<double>("z_max", config_.z_max, 1.0);

  pnh_.param<double>("r_min", config_.r_min, 0.10);
  pnh_.param<double>("r_max", config_.r_max, 0.15);

  pnh_.param<double>("cluster_tolerance", config_.cluster_tolerance, 0.10);
  pnh_.param<int>("min_cluster_size", config_.min_cluster_size, 25);
  pnh_.param<int>("max_cluster_size", config_.max_cluster_size, 2000);

  pnh_.param<int>("ransac_iters", config_.ransac_iters, 200);
  pnh_.param<double>("circle_inlier_th", config_.circle_inlier_th, 0.02);
  pnh_.param<int>("min_circle_inliers", config_.min_circle_inliers, 20);

  pnh_.param<bool>("enable_kf", config_.enable_kf, true);
  pnh_.param<int>("max_missed_frames", config_.max_missed_frames, 8);
  pnh_.param<double>("roi_base", config_.roi_base, 1.0);
  pnh_.param<double>("roi_vel_gain", config_.roi_vel_gain, 3.0);

  pnh_.param<double>("kf_q_pos", config_.kf_q_pos, 0.5);
  pnh_.param<double>("kf_q_vel", config_.kf_q_vel, 2.0);
  pnh_.param<double>("kf_r_meas", config_.kf_r_meas, 0.05);

  pnh_.param<bool>("publish_marker", config_.publish_marker, true);
  pnh_.param<double>("marker_z", config_.marker_z, 0.0);

  pnh_.param<int>("tcp_port", tcp_port_, 5050);

  for (auto& r : modbus_registers_) r.store(0, std::memory_order_relaxed);
  modbus_registers_[REG_DISTANCE].store(1000, std::memory_order_relaxed);
  modbus_registers_[REG_ANGLE].store(0, std::memory_order_relaxed);
}

void TargetTrackerNode::setupSubscribers()
{
  sub_ = nh_.subscribe(config_.input_topic, 1, &TargetTrackerNode::cloudCb, this);
}

void TargetTrackerNode::setupPublishers()
{
  pub_target_ = nh_.advertise<geometry_msgs::Pose2D>("/target_pose2d", 10);
  pub_vehicle_ = nh_.advertise<geometry_msgs::Pose2D>("/vehicle_wrt_target", 10);
  pub_status_ = nh_.advertise<std_msgs::UInt8>("/target_status", 10);
  if (config_.publish_marker)
  {
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/target_marker", 10);
  }

  ROS_INFO("TCP Port: %d", tcp_port_);
  if (initTCPServer()) {
      ROS_INFO("TCP server initialized on port %d", tcp_port_);
  } else {
      ROS_WARN("Failed to initialize TCP server on port %d", tcp_port_);
  }

  heartbeat_thread_ = std::thread(&TargetTrackerNode::heartbeatThread, this);
}

void TargetTrackerNode::cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  auto t0 = ros::WallTime::now();

  double dt = 0.1;
  if (!last_stamp_.isZero())
  {
    dt = (msg->header.stamp - last_stamp_).toSec();
    if (!(dt > 1e-4 && dt < 1.0)) dt = 0.1;
  }
  last_stamp_ = msg->header.stamp;

  if (config_.enable_kf && kf_.inited())
    kf_.predict(dt, config_.kf_q_pos, config_.kf_q_vel);

  const bool has_track = config_.enable_kf && kf_.inited();
  const double pred_x = has_track ? kf_.x() : 0.0;
  const double pred_y = has_track ? kf_.y() : 0.0;
  const double pred_speed = has_track ? kf_.speed() : 0.0;

  double roi = config_.roi_base;
  if (has_track)
    roi = config_.roi_base + config_.roi_vel_gain * pred_speed * dt;

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  filtered->reserve(1000);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> it_i(*msg, "intensity");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i)
  {
    const float x = *it_x;
    const float y = *it_y;
    const float z = *it_z;
    const float inten = *it_i;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    if (inten <= config_.intensity_threshold) continue;
    if (config_.use_z_filter && (z < config_.z_min || z > config_.z_max)) continue;

    if (has_track)
    {
      if (x < pred_x - roi || x > pred_x + roi) continue;
      if (y < pred_y - roi || y > pred_y + roi) continue;
    }

    pcl::PointXYZI p;
    p.x = x; p.y = y; p.z = z; p.intensity = inten;
    filtered->push_back(p);
  }

  filtered->is_dense = true;

  ROS_WARN_THROTTLE(1.0, "filtered=%zu has_track=%d pred=(%.2f,%.2f) roi=%.2f speed=%.2f",
                    filtered->size(), has_track, pred_x, pred_y, roi, pred_speed);

  if (filtered->size() < static_cast<size_t>(config_.min_cluster_size))
  {
    ROS_WARN_THROTTLE(1.0, "filtered points < min_cluster_size: %zu < %d",
                      filtered->size(), config_.min_cluster_size);
    handleNoMeasurement();
    publishOutputs(msg->header.frame_id, msg->header.stamp, false);
    return;
  }

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  tree->setInputCloud(filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(config_.cluster_tolerance);
  ec.setMinClusterSize(config_.min_cluster_size);
  ec.setMaxClusterSize(config_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(filtered);
  ec.extract(cluster_indices);

  if (cluster_indices.empty())
  {
    ROS_WARN_THROTTLE(1.0, "no clusters found");
    handleNoMeasurement();
    publishOutputs(msg->header.frame_id, msg->header.stamp, false);
    return;
  }

  bool found = false;
  double best_x = 0, best_y = 0;
  int best_inliers = -1;
  double best_err = 1e9;
  double best_pred_dist = 1e9;

  for (const auto& idx : cluster_indices)
  {
    std::vector<Eigen::Vector2d> pts2;
    pts2.reserve(idx.indices.size());

    double cx = 0, cy = 0;
    for (int id : idx.indices)
    {
      const auto& p = (*filtered)[id];
      pts2.emplace_back(p.x, p.y);
      cx += p.x; cy += p.y;
    }
    cx /= static_cast<double>(pts2.size());
    cy /= static_cast<double>(pts2.size());

    CircleFitResult c = ransacCircle2D(pts2, config_.ransac_iters, config_.r_min, config_.r_max, config_.circle_inlier_th);
    if (!c.ok) continue;
    if (c.inliers < config_.min_circle_inliers) continue;

    double pred_dist = 0.0;
    if (has_track) pred_dist = std::hypot(c.cx - pred_x, c.cy - pred_y);

    bool better = false;
    if (!found) better = true;
    else if (c.inliers > best_inliers) better = true;
    else if (c.inliers == best_inliers && c.mean_abs_err < best_err) better = true;
    else if (c.inliers == best_inliers && std::fabs(c.mean_abs_err - best_err) < 1e-6 && pred_dist < best_pred_dist) better = true;

    if (better)
    {
      found = true;
      best_x = c.cx;
      best_y = c.cy;
      best_inliers = c.inliers;
      best_err = c.mean_abs_err;
      best_pred_dist = pred_dist;
    }
  }

  if (!found)
  {
    ROS_WARN_THROTTLE(1.0, "no valid circle found");
    handleNoMeasurement();
    publishOutputs(msg->header.frame_id, msg->header.stamp, false);
    return;
  }

  if (config_.enable_kf)
  {
    if (!kf_.inited())
      kf_.reset(best_x, best_y);
    else
      kf_.update(best_x, best_y, config_.kf_r_meas);
  }

  missed_ = 0;
  last_meas_x_ = best_x;
  last_meas_y_ = best_y;
  have_last_meas_ = true;

  auto ms = (ros::WallTime::now() - t0).toSec() * 1000.0;
  ROS_WARN_THROTTLE(1.0, "FOUND: target=(%.2f,%.2f) inliers=%d err=%.3f proc=%.2fms clusters=%zu missed=%d",
                    best_x, best_y, best_inliers, best_err, ms, cluster_indices.size(), missed_);

  publishOutputs(msg->header.frame_id, msg->header.stamp, true);
}

void TargetTrackerNode::handleNoMeasurement()
{
  missed_++;
  if (missed_ > config_.max_missed_frames)
  {
    if (config_.enable_kf) kf_ = KalmanFilterCV();
    have_last_meas_ = false;
  }
}

void TargetTrackerNode::publishOutputs(const std::string& frame_id, const ros::Time& stamp, bool measured)
{
  std_msgs::UInt8 st;
  if (config_.enable_kf && kf_.inited())
  {
    if (measured) st.data = 1;
    else st.data = 2;
  }
  else
  {
    st.data = 0;
  }
  pub_status_.publish(st);

  if (!(config_.enable_kf && kf_.inited()))
    return;

  const double tx = kf_.x();
  const double ty = kf_.y();

  geometry_msgs::Pose2D target;
  target.x = tx;
  target.y = ty;
  target.theta = 0.0;
  pub_target_.publish(target);

  geometry_msgs::Pose2D veh;
  veh.x = -tx;
  veh.y = -ty;
  veh.theta = 0.0;
  pub_vehicle_.publish(veh);

  // 发布TCP数据：雷达相对于小车的坐标 (tx, ty 在这里是标靶在雷达坐标系，需要取反)
  // 标靶在雷达坐标系: (tx, ty)
  // 雷达在小车坐标系: (-tx, -ty)
  sendTCPData(-tx, -ty);

  const char* status_str = measured ? "FOUND" : "PREDICT_ONLY";
  ROS_WARN_THROTTLE(1.0, "STATUS=%d(%s) target=(%.2f,%.2f) missed=%d",
                    static_cast<int>(st.data), status_str, tx, ty, missed_);

  if (config_.publish_marker)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = frame_id;
    mk.header.stamp = stamp;
    mk.ns = "reflector_target";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = tx;
    mk.pose.position.y = ty;
    mk.pose.position.z = config_.marker_z;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.2;
    mk.color.b = 0.2;
    pub_marker_.publish(mk);
  }
}

bool TargetTrackerNode::initTCPServer()
{
  tcp_server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_server_fd_ < 0) {
    ROS_ERROR("Failed to create socket: %s", strerror(errno));
    return false;
  }

  int opt = 1;
  setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
  setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

  setSockBuf(tcp_server_fd_, 512 * 1024, 512 * 1024);
  setNoDelay(tcp_server_fd_);
  setNonBlocking(tcp_server_fd_);

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(tcp_port_);

  if (bind(tcp_server_fd_, (struct sockaddr*)&server_addr,
           sizeof(server_addr)) < 0) {
    ROS_ERROR("Failed to bind to port %d: %s", tcp_port_, strerror(errno));
    close(tcp_server_fd_);
    tcp_server_fd_ = -1;
    return false;
  }

  if (listen(tcp_server_fd_, 64) < 0) {
    ROS_ERROR("Failed to listen on socket: %s", strerror(errno));
    close(tcp_server_fd_);
    tcp_server_fd_ = -1;
    return false;
  }

  tcp_server_running_.store(true, std::memory_order_release);
  tcp_server_thread_ = std::thread(&TargetTrackerNode::tcpServerThread, this);
  return true;
}

void TargetTrackerNode::closeTCPServer()
{
  bool expected = true;
  if (!tcp_server_running_.compare_exchange_strong(expected, false)) {
  }

  if (tcp_server_fd_ >= 0) {
    shutdown(tcp_server_fd_, SHUT_RDWR);
    close(tcp_server_fd_);
    tcp_server_fd_ = -1;
  }

  if (tcp_server_thread_.joinable()) {
    tcp_server_thread_.join();
  }

  for (int fd : tcp_client_fds_) {
    shutdown(fd, SHUT_RDWR);
    close(fd);
  }
  tcp_client_fds_.clear();
  client_rx_caches_.clear();
}

void TargetTrackerNode::removeClient(int client_fd)
{
  auto it = std::find(tcp_client_fds_.begin(), tcp_client_fds_.end(), client_fd);
  if (it != tcp_client_fds_.end()) tcp_client_fds_.erase(it);
  client_rx_caches_.erase(client_fd);

  shutdown(client_fd, SHUT_RDWR);
  close(client_fd);

  ROS_WARN("TCP client removed (fd=%d), total clients: %zu", client_fd,
           tcp_client_fds_.size());
}

void TargetTrackerNode::tcpServerThread()
{
  ROS_INFO("TCP server thread started, port=%d", tcp_port_);

  while (tcp_server_running_.load(std::memory_order_acquire)) {
    fd_set read_fds;
    FD_ZERO(&read_fds);

    int max_fd = -1;
    if (tcp_server_fd_ >= 0) {
      FD_SET(tcp_server_fd_, &read_fds);
      max_fd = tcp_server_fd_;
    }

    for (int fd : tcp_client_fds_) {
      FD_SET(fd, &read_fds);
      if (fd > max_fd) max_fd = fd;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200 * 1000;

    int ret = select(max_fd + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (!tcp_server_running_.load()) break;
      if (errno == EINTR) continue;
      ROS_WARN("select error: %s", strerror(errno));
      continue;
    }
    if (!tcp_server_running_.load()) break;

    if (tcp_server_fd_ >= 0 && FD_ISSET(tcp_server_fd_, &read_fds)) {
      while (true) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(tcp_server_fd_, (struct sockaddr*)&client_addr,
                              &client_len);
        if (client_fd < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) break;
          if (errno == EINTR) continue;
          ROS_WARN("accept error: %s", strerror(errno));
          break;
        }

        setNonBlocking(client_fd);
        setSockBuf(client_fd, 512 * 1024, 512 * 1024);
        setNoDelay(client_fd);
        setKeepAlive(client_fd);

        tcp_client_fds_.push_back(client_fd);
        client_rx_caches_[client_fd].clear();

        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, ip, INET_ADDRSTRLEN);
        ROS_INFO("TCP client connected: %s:%d (fd=%d), total=%zu", ip,
                 ntohs(client_addr.sin_port), client_fd,
                 tcp_client_fds_.size());
      }
    }

    for (size_t i = 0; i < tcp_client_fds_.size(); ) {
      int client_fd = tcp_client_fds_[i];
      if (!FD_ISSET(client_fd, &read_fds)) {
        ++i;
        continue;
      }

      bool need_close = false;
      uint8_t buffer[512];

      auto& rx_cache = client_rx_caches_[client_fd];

      while (true) {
        ssize_t n = recv(client_fd, buffer, sizeof(buffer), 0);
        if (n > 0) {
          rx_cache.insert(rx_cache.end(), buffer, buffer + n);
          continue;
        }
        if (n == 0) {
          need_close = true;
          break;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        need_close = true;
        break;
      }

      if (need_close) {
        removeClient(client_fd);
        continue;
      }

      while (true) {
        if (rx_cache.size() < 6) break;

        uint16_t protocol_id = (rx_cache[2] << 8) | rx_cache[3];
        uint16_t length_field = (rx_cache[4] << 8) | rx_cache[5];

        if (protocol_id != 0) {
          rx_cache.erase(rx_cache.begin());
          continue;
        }
        if (length_field < 2 || length_field > 253) {
          rx_cache.erase(rx_cache.begin());
          continue;
        }

        size_t total = 6 + static_cast<size_t>(length_field);
        if (rx_cache.size() < total) break;

        std::vector<uint8_t> frame(rx_cache.begin(),
                                     rx_cache.begin() + total);
        rx_cache.erase(rx_cache.begin(), rx_cache.begin() + total);

        handleModbusRequest(client_fd, frame.data(),
                            static_cast<ssize_t>(frame.size()));

        if (std::find(tcp_client_fds_.begin(), tcp_client_fds_.end(),
                      client_fd) == tcp_client_fds_.end()) {
          break;
        }
      }

      ++i;
    }
  }

  ROS_INFO("TCP server thread stopped");
}

void TargetTrackerNode::heartbeatThread()
{
  ROS_INFO("Heartbeat thread started (20ms)");
  while (tcp_server_running_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    uint16_t hb = heartbeat_.fetch_add(1) + 1;
    if (hb == 0) {
      heartbeat_.store(1);
      hb = 1;
    }
    modbus_registers_[REG_HEARTBEAT].store(hb, std::memory_order_relaxed);
  }
  ROS_INFO("Heartbeat thread stopped");
}

void TargetTrackerNode::sendTCPData(double lidar_x, double lidar_y)
{
  int32_t x_mm32 = static_cast<int32_t>(std::llround(lidar_x * 1000.0));
  int32_t y_mm32 = static_cast<int32_t>(std::llround(lidar_y * 1000.0));

  if (x_mm32 > 32767) x_mm32 = 32767;
  if (x_mm32 < -32768) x_mm32 = -32768;
  if (y_mm32 > 32767) y_mm32 = 32767;
  if (y_mm32 < -32768) y_mm32 = -32768;

  int16_t x_mm = static_cast<int16_t>(x_mm32);
  int16_t y_mm = static_cast<int16_t>(y_mm32);

  modbus_registers_[REG_X_COORD].store(static_cast<uint16_t>(x_mm), std::memory_order_relaxed);
  modbus_registers_[REG_Y_COORD].store(static_cast<uint16_t>(y_mm), std::memory_order_relaxed);

  // 距离和角度保持固定值
  modbus_registers_[REG_DISTANCE].store(1000, std::memory_order_relaxed);  // 固定1000
  modbus_registers_[REG_ANGLE].store(0, std::memory_order_relaxed);        // 固定0
}

void TargetTrackerNode::handleModbusRequest(int client_fd, const uint8_t* request, ssize_t length)
{
  if (length < 8) return;

  uint16_t protocol_id = (request[2] << 8) | request[3];
  uint16_t length_field = (request[4] << 8) | request[5];
  uint8_t unit_id = request[6];
  uint8_t function_code = request[7];

  if (protocol_id != 0) return;
  if (length != static_cast<ssize_t>(6 + length_field)) return;

  uint8_t response[260];
  ssize_t response_len = 0;

  response[0] = request[0];
  response[1] = request[1];
  response[2] = request[2];
  response[3] = request[3];
  response[6] = unit_id;

  switch (function_code) {
    case 0x03: {
      if (length < 12) return;
      uint16_t start_addr = (request[8] << 8) | request[9];
      uint16_t reg_count = (request[10] << 8) | request[11];

      if (reg_count == 0 || reg_count > 125) {
        response[7] = 0x83;
        response[8] = 0x03;
        response_len = 9;
        break;
      }
      if (static_cast<size_t>(start_addr) + reg_count > REG_COUNT) {
        response[7] = 0x83;
        response[8] = 0x02;
        response_len = 9;
        break;
      }

      response[7] = 0x03;
      response[8] = static_cast<uint8_t>(reg_count * 2);
      response_len = 9;

      for (uint16_t i = 0; i < reg_count; ++i) {
        uint16_t v = modbus_registers_[start_addr + i].load(std::memory_order_relaxed);
        response[response_len++] = static_cast<uint8_t>((v >> 8) & 0xFF);
        response[response_len++] = static_cast<uint8_t>(v & 0xFF);
      }
      break;
    }

    case 0x06: {
      if (length < 12) return;
      uint16_t reg_addr = (request[8] << 8) | request[9];
      uint16_t reg_value = (request[10] << 8) | request[11];

      if (reg_addr >= REG_COUNT) {
        response[7] = 0x86;
        response[8] = 0x02;
        response_len = 9;
        break;
      }

      modbus_registers_[reg_addr].store(reg_value, std::memory_order_relaxed);

      response[7] = 0x06;
      response[8] = request[8];
      response[9] = request[9];
      response[10] = request[10];
      response[11] = request[11];
      response_len = 12;
      break;
    }

    default: {
      response[7] = function_code | 0x80;
      response[8] = 0x01;
      response_len = 9;
      break;
    }
  }

  response[4] = static_cast<uint8_t>(((response_len - 6) >> 8) & 0xFF);
  response[5] = static_cast<uint8_t>((response_len - 6) & 0xFF);

  const int SEND_DEADLINE_MS = 100;
  bool ok = sendAllWithDeadlineMs(client_fd, response,
                                  static_cast<size_t>(response_len),
                                  SEND_DEADLINE_MS);
  if (!ok) {
    ROS_WARN("send Modbus response failed/timeout (fd=%d, errno=%s). drop client.",
             client_fd, strerror(errno));
    removeClient(client_fd);
  }
}

} // namespace tracker
