#include "lidar_target_detection/lidar_target_detector.h"

#include <errno.h>
#include <fcntl.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sys/select.h>

#include <chrono>
#include <iomanip>
#include <sstream>

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

    // Linux-only keepalive tuning（在 Docker/ARM64 Linux OK）
    int keepidle = 30; // 30s idle -> probe
    int keepintvl = 5; // interval 5s
    int keepcnt = 3;   // 3 probes
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));
}

// 带 deadline 的 sendAll，防止单个客户端拖死整个 Modbus 服务
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
            return false; // timeout or select error
        }
        return false; // other fatal send error
    }
    return true;
}

} // namespace

LidarTargetDetector::LidarTargetDetector(ros::NodeHandle& nh,
                                         ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      tcp_server_fd_(-1),
      tcp_port_(5050),
      tcp_server_running_(false),
      heartbeat_(0)
{
    private_nh_.param("tcp_port", tcp_port_, 5050);
    private_nh_.param("intensity_threshold", intensity_threshold_, 150.0);
    private_nh_.param("cluster_tolerance", cluster_tolerance_, 0.5);
    private_nh_.param("min_cluster_size", min_cluster_size_, 10);
    private_nh_.param("max_cluster_size", max_cluster_size_, 500);
    private_nh_.param("min_arc_angle", min_arc_angle_, 10.0);
    private_nh_.param("max_arc_angle", max_arc_angle_, 360.0);
    private_nh_.param("max_rmse_threshold", max_rmse_threshold_, 0.05);
    private_nh_.param("min_target_radius", min_target_radius_, 0.01);
    private_nh_.param("max_target_radius", max_target_radius_, 0.5);

    cloud_sub_ = nh_.subscribe("input_cloud", 1,
                               &LidarTargetDetector::cloudCallback, this);
    target_pub_ =
        nh_.advertise<geometry_msgs::PointStamped>("target_position", 1);
    filtered_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("clusters", 1);

    state_ = Eigen::Vector4d::Zero();
    P_ = Eigen::Matrix4d::Identity() * 10.0;
    H_ = Eigen::Matrix<double, 2, 4>::Zero();
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    Q_ = Eigen::Matrix4d::Identity() * 0.01;
    R_ = Eigen::Matrix2d::Identity() * 0.001;
    last_update_time_ = ros::Time::now();
    first_detection_ = true;

    for (auto& r : modbus_registers_) r.store(0, std::memory_order_relaxed);
    modbus_registers_[REG_DISTANCE].store(1000, std::memory_order_relaxed);
    modbus_registers_[REG_ANGLE].store(0, std::memory_order_relaxed);

    ROS_INFO("Lidar Target Detector initialized:");
    ROS_INFO("  TCP Port: %d", tcp_port_);
    ROS_INFO("  Intensity threshold: %.1f", intensity_threshold_);
    ROS_INFO("  Cluster tolerance: %.3f m", cluster_tolerance_);
    ROS_INFO("  Min/Max cluster size: %d/%d", min_cluster_size_,
             max_cluster_size_);
    ROS_INFO("  Arc angle range: %.1f-%.1f degrees", min_arc_angle_,
             max_arc_angle_);
    ROS_INFO("  Max RMSE threshold: %.3f", max_rmse_threshold_);
    ROS_INFO("  Target radius range: %.3f-%.3f m", min_target_radius_,
             max_target_radius_);

    if (initTCPServer()) {
        ROS_INFO("TCP server initialized on port %d", tcp_port_);
    } else {
        ROS_WARN("Failed to initialize TCP server on port %d", tcp_port_);
    }

    heartbeat_thread_ =
        std::thread(&LidarTargetDetector::heartbeatThread, this);
}

LidarTargetDetector::~LidarTargetDetector()
{
    closeTCPServer();
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
}

bool LidarTargetDetector::initTCPServer()
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
    tcp_server_thread_ =
        std::thread(&LidarTargetDetector::tcpServerThread, this);
    return true;
}

void LidarTargetDetector::closeTCPServer()
{
    bool expected = true;
    if (!tcp_server_running_.compare_exchange_strong(expected, false)) {
        // already stopped
    }

    // 先关闭 listen fd，唤醒 select
    if (tcp_server_fd_ >= 0) {
        shutdown(tcp_server_fd_, SHUT_RDWR);
        close(tcp_server_fd_);
        tcp_server_fd_ = -1;
    }

    if (tcp_server_thread_.joinable()) {
        tcp_server_thread_.join();
    }

    // 线程退出后再清理客户端资源
    for (int fd : tcp_client_fds_) {
        shutdown(fd, SHUT_RDWR);
        close(fd);
    }
    tcp_client_fds_.clear();
    client_rx_caches_.clear();
}

void LidarTargetDetector::removeClient(int client_fd)
{
    auto it =
        std::find(tcp_client_fds_.begin(), tcp_client_fds_.end(), client_fd);
    if (it != tcp_client_fds_.end()) tcp_client_fds_.erase(it);
    client_rx_caches_.erase(client_fd);

    shutdown(client_fd, SHUT_RDWR);
    close(client_fd);

    ROS_WARN("TCP client removed (fd=%d), total clients: %zu", client_fd,
             tcp_client_fds_.size());
}

void LidarTargetDetector::tcpServerThread()
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

        // select 监听所有 client
        for (int fd : tcp_client_fds_) {
            FD_SET(fd, &read_fds);
            if (fd > max_fd) max_fd = fd;
        }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200 * 1000; // 200ms：低延迟且可快速退出

        int ret = select(max_fd + 1, &read_fds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (!tcp_server_running_.load()) break;
            if (errno == EINTR) continue;
            ROS_WARN("select error: %s", strerror(errno));
            continue;
        }
        if (!tcp_server_running_.load()) break;

        // 1) accept draining
        if (tcp_server_fd_ >= 0 && FD_ISSET(tcp_server_fd_, &read_fds)) {
            while (true) {
                struct sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
                int client_fd =
                    accept(tcp_server_fd_, (struct sockaddr*)&client_addr,
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

        // 2) recv draining for each ready client
        // 注意：遍历时可能 removeClient，使用 index 方式更安全
        for (size_t i = 0; i < tcp_client_fds_.size(); /*++i in loop*/) {
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
                    continue; // draining
                }
                if (n == 0) { // peer closed
                    need_close = true;
                    break;
                }
                if (n < 0 && errno == EINTR) continue;
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                // other error
                need_close = true;
                break;
            }

            if (need_close) {
                removeClient(client_fd);
                // removeClient 会改变 tcp_client_fds_，此时不要 ++i
                continue;
            }

            // 3) Modbus TCP framing
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

                // 如果在 handle 里被移除，rx_cache
                // 引用已经失效；安全起见检查是否还在列表
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

void LidarTargetDetector::heartbeatThread()
{
    ROS_INFO("Heartbeat thread started (20ms)");
    while (tcp_server_running_.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        uint16_t hb = heartbeat_.fetch_add(1) + 1;
        if (hb == 0) { // wrap
            heartbeat_.store(1);
            hb = 1;
        }
        modbus_registers_[REG_HEARTBEAT].store(hb, std::memory_order_relaxed);
    }
    ROS_INFO("Heartbeat thread stopped");
}

void LidarTargetDetector::sendTCPData(
    const geometry_msgs::PointStamped& position)
{
    // 更新 Modbus 寄存器（原子写）
    int32_t x_mm32 =
        static_cast<int32_t>(std::llround(position.point.x * 1000.0));
    int32_t y_mm32 =
        static_cast<int32_t>(std::llround(position.point.y * 1000.0));

    // 截断到 int16 范围，避免溢出导致奇怪值
    if (x_mm32 > 32767) x_mm32 = 32767;
    if (x_mm32 < -32768) x_mm32 = -32768;
    if (y_mm32 > 32767) y_mm32 = 32767;
    if (y_mm32 < -32768) y_mm32 = -32768;

    int16_t x_mm = static_cast<int16_t>(x_mm32);
    int16_t y_mm = static_cast<int16_t>(y_mm32);

    modbus_registers_[REG_X_COORD].store(static_cast<uint16_t>(x_mm),
                                         std::memory_order_relaxed);
    modbus_registers_[REG_Y_COORD].store(static_cast<uint16_t>(y_mm),
                                         std::memory_order_relaxed);
}

void LidarTargetDetector::handleModbusRequest(int client_fd,
                                              const uint8_t* request,
                                              ssize_t length)
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

    // MBAP: transaction + protocol
    response[0] = request[0];
    response[1] = request[1];
    response[2] = request[2];
    response[3] = request[3];
    response[6] = unit_id;

    switch (function_code) {
        case 0x03: { // Read Holding Registers
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
                uint16_t v = modbus_registers_[start_addr + i].load(
                    std::memory_order_relaxed);
                response[response_len++] =
                    static_cast<uint8_t>((v >> 8) & 0xFF);
                response[response_len++] = static_cast<uint8_t>(v & 0xFF);
            }
            break;
        }

        case 0x06: { // Write Single Register
            if (length < 12) return;
            uint16_t reg_addr = (request[8] << 8) | request[9];
            uint16_t reg_value = (request[10] << 8) | request[11];

            if (reg_addr >= REG_COUNT) {
                response[7] = 0x86;
                response[8] = 0x02;
                response_len = 9;
                break;
            }

            modbus_registers_[reg_addr].store(reg_value,
                                              std::memory_order_relaxed);

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
            response[8] = 0x01; // illegal function
            response_len = 9;
            break;
        }
    }

    // length = UnitId(1) + PDU(response_len-7)
    response[4] = static_cast<uint8_t>(((response_len - 6) >> 8) & 0xFF);
    response[5] = static_cast<uint8_t>((response_len - 6) & 0xFF);

    // 关键：短 deadline，避免一个坏连接把整个线程卡死
    const int SEND_DEADLINE_MS = 100;
    bool ok = sendAllWithDeadlineMs(client_fd, response,
                                    static_cast<size_t>(response_len),
                                    SEND_DEADLINE_MS);
    if (!ok) {
        ROS_WARN(
            "send Modbus response failed/timeout (fd=%d, errno=%s). drop "
            "client.",
            client_fd, strerror(errno));
        removeClient(client_fd);
    }
}

// ===== 点云处理逻辑（基本保持你的原始版本） =====

void LidarTargetDetector::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    auto t_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<PointXYZIRT>::Ptr cloud(new pcl::PointCloud<PointXYZIRT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (cloud->empty()) {
        ROS_WARN("Received empty point cloud");
        return;
    }

    // 动态调整强度阈值:移动时降低阈值,保留更多点
    double speed = std::sqrt(state_(2) * state_(2) + state_(3) * state_(3));
    double dynamic_threshold = intensity_threshold_;
    if (speed > 0.1) {
        dynamic_threshold = intensity_threshold_ * moving_intensity_factor_; // 移动时降低阈值
    }

    pcl::PointCloud<PointXYZIRT>::Ptr filtered_cloud =
        filterByIntensityDynamic(cloud, dynamic_threshold);
    if (filtered_cloud->empty()) {
        static ros::Time last_warn;
        ros::Time now = ros::Time::now();
        if ((now - last_warn).toSec() > 2.0) {
            ROS_WARN_THROTTLE(2.0, "No points above intensity threshold %.1f (dynamic: %.1f)",
                             intensity_threshold_, dynamic_threshold);
            last_warn = now;
        }
        return;
    }

    std::vector<pcl::PointCloud<PointXYZIRT>::Ptr> clusters =
        extractClusters(filtered_cloud);
    if (clusters.empty()) {
        static ros::Time last_warn;
        ros::Time now = ros::Time::now();
        if ((now - last_warn).toSec() > 2.0) {
            ROS_WARN("No clusters found in filtered cloud");
            last_warn = now;
        }
        return;
    }

    pcl::PointCloud<PointXYZIRT>::Ptr highest_intensity_cluster = nullptr;
    double max_avg_intensity = 0.0;

    for (const auto& cluster : clusters) {
        if (static_cast<int>(cluster->size()) < min_cluster_size_) continue;
        double avg_intensity = 0.0;
        for (const auto& p : cluster->points) avg_intensity += p.intensity;
        avg_intensity /= std::max<size_t>(1, cluster->size());
        if (avg_intensity > max_avg_intensity) {
            max_avg_intensity = avg_intensity;
            highest_intensity_cluster = cluster;
        }
    }

    geometry_msgs::PointStamped best_target;
    bool target_found = false;
    if (highest_intensity_cluster &&
        fitArcAndGetCenter(highest_intensity_cluster, best_target)) {
        target_found = true;
        ROS_INFO_THROTTLE(2.0, "Target detected at (%.3f, %.3f)",
                          best_target.point.x, best_target.point.y);
    } else {
        static ros::Time last_fail_warn;
        ros::Time now = ros::Time::now();
        if ((now - last_fail_warn).toSec() > 3.0) {
            ROS_WARN("Target fitting failed! Check arc_angle/radius thresholds.");
            last_fail_warn = now;
        }
    }

    if (target_found) {
        updateKalmanFilter(best_target);

        geometry_msgs::PointStamped filtered_position;
        filtered_position.header = cloud_msg->header;
        filtered_position.point.x = state_(0);
        filtered_position.point.y = state_(1);
        filtered_position.point.z = 0.0;

        target_pub_.publish(filtered_position);
        sendTCPData(filtered_position);
        last_update_time_ = ros::Time::now();
    } else {
        predictOnly();

        geometry_msgs::PointStamped predicted_position;
        predicted_position.header = cloud_msg->header;
        predicted_position.point.x = state_(0);
        predicted_position.point.y = state_(1);
        predicted_position.point.z = 0.0;

        target_pub_.publish(predicted_position);
        sendTCPData(predicted_position);

        static ros::Time last_warn;
        ros::Time now = ros::Time::now();
        if ((now - last_warn).toSec() > 1.0) {
            ROS_WARN_THROTTLE(2.0, "Target not detected! Using Kalman prediction at (%.3f, %.3f). Check thresholds.",
                             predicted_position.point.x, predicted_position.point.y);
        }
    }

    // 发布过滤点云（单色）
    pcl::PointCloud<pcl::PointXYZRGB> single_color_cloud;
    single_color_cloud.reserve(filtered_cloud->size());
    for (const auto& p : filtered_cloud->points) {
        pcl::PointXYZRGB cp;
        cp.x = p.x;
        cp.y = p.y;
        cp.z = p.z;
        cp.r = 255;
        cp.g = 0;
        cp.b = 0;
        single_color_cloud.push_back(cp);
    }
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(single_color_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = cloud_msg->header;
    filtered_cloud_pub_.publish(filtered_cloud_msg);

    publishClustersVisualization(clusters, cloud_msg->header);
}

pcl::PointCloud<PointXYZIRT>::Ptr LidarTargetDetector::filterByIntensity(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    pcl::PointCloud<PointXYZIRT>::Ptr out(new pcl::PointCloud<PointXYZIRT>);
    out->reserve(cloud->size());
    for (const auto& p : cloud->points) {
        if (p.intensity >= intensity_threshold_) out->push_back(p);
    }
    out->width = out->size();
    out->height = 1;
    out->is_dense = true;
    return out;
}

pcl::PointCloud<PointXYZIRT>::Ptr LidarTargetDetector::filterByIntensityDynamic(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, double threshold)
{
    pcl::PointCloud<PointXYZIRT>::Ptr out(new pcl::PointCloud<PointXYZIRT>);
    out->reserve(cloud->size());
    for (const auto& p : cloud->points) {
        if (p.intensity >= threshold) out->push_back(p);
    }
    out->width = out->size();
    out->height = 1;
    out->is_dense = true;
    return out;
}

std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>
LidarTargetDetector::extractClusters(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    std::vector<pcl::PointCloud<PointXYZIRT>::Ptr> clusters;

    pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    xyz_cloud->reserve(cloud->size());
    for (const auto& p : cloud->points) {
        pcl::PointXYZI q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = p.intensity;
        xyz_cloud->push_back(q);
    }

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(xyz_cloud);

    // 动态调整聚类参数:根据Kalman速度估计判断是否为移动目标
    double speed = std::sqrt(state_(2) * state_(2) + state_(3) * state_(3));
    bool is_moving = speed > 0.1; // 速度 > 0.1m/s 认为在移动

    // 移动时:增大容差和聚类尺寸范围
    double dynamic_tolerance = is_moving ? cluster_tolerance_ * moving_tolerance_factor_ : cluster_tolerance_;
    int dynamic_min_size = is_moving ? static_cast<int>(min_cluster_size_ * moving_min_cluster_factor_) : min_cluster_size_;
    int dynamic_max_size = is_moving ? static_cast<int>(max_cluster_size_ * moving_max_cluster_factor_) : max_cluster_size_;

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(dynamic_tolerance);
    ec.setMinClusterSize(dynamic_min_size);
    ec.setMaxClusterSize(dynamic_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(xyz_cloud);
    ec.extract(cluster_indices);

    // 记录调试信息
    static ros::Time last_log;
    ros::Time now = ros::Time::now();
    if ((now - last_log).toSec() > 2.0) {
        ROS_INFO("Clustering: %d clusters found, speed=%.2fm/s, tolerance=%.3fm, size=[%d,%d]",
                 (int)cluster_indices.size(), speed, dynamic_tolerance,
                 dynamic_min_size, dynamic_max_size);
        last_log = now;
    }

    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<PointXYZIRT>::Ptr c(new pcl::PointCloud<PointXYZIRT>);
        c->reserve(indices.indices.size());
        for (int idx : indices.indices) {
            c->push_back((*cloud)[idx]);
        }
        c->width = c->size();
        c->height = 1;
        c->is_dense = true;
        clusters.push_back(c);
    }

    return clusters;
}

bool LidarTargetDetector::fitArcAndGetCenter(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cluster,
    geometry_msgs::PointStamped& center)
{
    if (cluster->size() < 10) {
        static ros::Time last_log;
        ros::Time now = ros::Time::now();
        if ((now - last_log).toSec() > 3.0) {
            ROS_WARN_THROTTLE(3.0, "Fit failed: cluster size=%d < min=10", (int)cluster->size());
            last_log = now;
        }
        return false;
    }

    double sum_z = 0.0;
    for (const auto& p : cluster->points) sum_z += p.z;
    double avg_z = sum_z / static_cast<double>(cluster->size());

    const int n = static_cast<int>(cluster->size());
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& p : cluster->points) {
        sum_x += p.x;
        sum_y += p.y;
    }
    double mean_x = sum_x / n;
    double mean_y = sum_y / n;

    for (int i = 0; i < n; ++i) {
        double x = cluster->points[i].x - mean_x;
        double y = cluster->points[i].y - mean_y;
        A(i, 0) = 2.0 * x;
        A(i, 1) = 2.0 * y;
        A(i, 2) = 1.0;
        b(i) = x * x + y * y;
    }

    Eigen::Vector3d sol =
        A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    double center_x = sol(0) + mean_x;
    double center_y = sol(1) + mean_y;
    double radius = std::sqrt(sol(0) * sol(0) + sol(1) * sol(1) + sol(2));

    // 放宽半径阈值以适应移动场景和实际标靶尺寸
    if (radius < min_target_radius_ * fit_radius_min_factor_ || radius > max_target_radius_ * fit_radius_max_factor_) {
        static ros::Time last_log;
        ros::Time now = ros::Time::now();
        if ((now - last_log).toSec() > 3.0) {
            ROS_WARN_THROTTLE(3.0, "Fit failed: radius=%.3fm out of range [%.3f, %.3f]",
                             radius, min_target_radius_ * fit_radius_min_factor_, max_target_radius_ * fit_radius_max_factor_);
            last_log = now;
        }
        return false;
    }

    double sum_sq_error = 0.0;
    for (int i = 0; i < n; ++i) {
        double dx = cluster->points[i].x - center_x;
        double dy = cluster->points[i].y - center_y;
        double ar = std::sqrt(dx * dx + dy * dy);
        double e = (ar - radius);
        sum_sq_error += e * e;
    }
    double rmse = std::sqrt(sum_sq_error / n);
    // 放宽RMSE阈值以容忍移动时的点云畸变
    if (rmse > max_rmse_threshold_ * fit_rmse_factor_) {
        static ros::Time last_log;
        ros::Time now = ros::Time::now();
        if ((now - last_log).toSec() > 3.0) {
            ROS_WARN_THROTTLE(3.0, "Fit failed: RMSE=%.4fm > threshold=%.4fm",
                             rmse, max_rmse_threshold_ * fit_rmse_factor_);
            last_log = now;
        }
        return false;
    }

    std::vector<double> angles;
    angles.reserve(cluster->size());
    for (const auto& p : cluster->points) {
        angles.push_back(std::atan2(p.y - center_y, p.x - center_x));
    }
    std::sort(angles.begin(), angles.end());

    double max_gap = 0.0;
    for (size_t i = 0; i < angles.size(); ++i) {
        double gap = angles[(i + 1) % angles.size()] - angles[i];
        if (gap < 0) gap += 2 * M_PI;
        max_gap = std::max(max_gap, gap);
    }

    double arc_angle = 2 * M_PI - max_gap;
    double arc_angle_deg = arc_angle * 180.0 / M_PI;

    // 放宽圆弧角度阈值 - 允许几乎完整的圆
    if (arc_angle_deg < min_arc_angle_ * fit_arc_min_factor_ || arc_angle_deg > max_arc_angle_) {
        static ros::Time last_log;
        ros::Time now = ros::Time::now();
        if ((now - last_log).toSec() > 3.0) {
            ROS_WARN_THROTTLE(3.0, "Fit failed: arc_angle=%.1f° out of range [%.1f, %.1f]",
                             arc_angle_deg, min_arc_angle_ * fit_arc_min_factor_, max_arc_angle_);
            last_log = now;
        }
        return false;
    }

    center.point.x = center_x;
    center.point.y = center_y;
    center.point.z = avg_z;
    return true;
}

void LidarTargetDetector::predictOnly()
{
    // 计算实际时间差
    ros::Time now = ros::Time::now();
    double dt = std::min((now - last_update_time_).toSec(), 0.5); // 限制最大dt=0.5s
    last_update_time_ = now;

    Eigen::Matrix4d F;
    F << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

    // 仅预测,不更新(增加不确定性)
    state_ = F * state_;
    P_ = F * P_ * F.transpose() + Q_ * fit_rmse_factor_; // 预测时增加更多不确定性
}

void LidarTargetDetector::updateKalmanFilter(
    const geometry_msgs::PointStamped& measurement)
{
    // 计算实际时间差
    ros::Time now = ros::Time::now();
    double dt = std::min((now - last_update_time_).toSec(), 1.0); // 限制最大dt=1.0s

    Eigen::Matrix4d F;
    F << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

    state_ = F * state_;
    P_ = F * P_ * F.transpose() + Q_ * dt; // Q随dt缩放

    Eigen::Vector2d z(measurement.point.x, measurement.point.y);
    Eigen::Vector2d y = z - H_ * state_;
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();

    state_ = state_ + K * y;
    P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
}

void LidarTargetDetector::publishClustersVisualization(
    const std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>& clusters,
    const std_msgs::Header& header)
{
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;

    std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> colors = {
        {255, 0, 0},   {0, 255, 0},   {0, 0, 255},   {255, 255, 0},
        {255, 0, 255}, {0, 255, 255}, {255, 128, 0}, {128, 0, 255},
        {128, 255, 0}, {0, 128, 255}};

    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        auto color = colors[i % colors.size()];
        for (const auto& p : cluster->points) {
            pcl::PointXYZRGB cp;
            cp.x = p.x;
            cp.y = p.y;
            cp.z = p.z;
            cp.r = std::get<0>(color);
            cp.g = std::get<1>(color);
            cp.b = std::get<2>(color);
            colored_cloud.push_back(cp);
        }
    }

    if (!colored_cloud.empty()) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(colored_cloud, msg);
        msg.header = header;
        clusters_pub_.publish(msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_target_detector");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    LidarTargetDetector detector(nh, private_nh);
    ros::spin();
    return 0;
}
