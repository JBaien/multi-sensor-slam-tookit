/**
 * @file target_detector.cpp
 * @brief 激光雷达反光标靶检测器实现
 */

#include "lidar_target_localization/target_detector.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>

#include <chrono>
#include <sys/select.h>
#include <netinet/tcp.h>

namespace {

/**
 * @brief 设置socket为非阻塞模式
 * @param fd socket文件描述符
 * @return 成功返回true，失败返回false
 */
static bool setNonBlocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return false;
    return true;
}

/**
 * @brief 设置socket发送和接收缓冲区大小
 * @param fd socket文件描述符
 * @param snd_bytes 发送缓冲区大小（字节）
 * @param rcv_bytes 接收缓冲区大小（字节）
 */
static void setSockBuf(int fd, int snd_bytes, int rcv_bytes)
{
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &snd_bytes, sizeof(snd_bytes));
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcv_bytes, sizeof(rcv_bytes));
}

/**
 * @brief 禁用Nagle算法，启用TCP_NODELAY
 * @param fd socket文件描述符
 */
static void setNoDelay(int fd)
{
    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
}

/**
 * @brief 启用TCP Keep-Alive机制
 * @param fd socket文件描述符
 *
 * Keep-Alive参数:
 * - keepidle: 30秒无数据后开始发送keepalive探测
 * - keepintvl: 探测失败后每5秒重试一次
 * - keepcnt: 最多重试3次后判定连接断开
 */
static void setKeepAlive(int fd)
{
    int keepalive = 1;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));

    int keepidle = 30;   // 30秒空闲后开始探测
    int keepintvl = 5;   // 探测间隔5秒
    int keepcnt = 3;     // 最多探测3次
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));
}

/**
 * @brief 带超时的数据发送函数
 * @param fd socket文件描述符
 * @param data 待发送数据
 * @param len 数据长度
 * @param deadline_ms 超时时间（毫秒）
 * @return 成功返回true，超时或失败返回false
 */
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
            // 计算剩余时间
            auto now = clock::now();
            int elapsed = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                      start)
                    .count());
            int remain = deadline_ms - elapsed;
            if (remain <= 0) return false;

            // 使用select等待socket可写
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

// ===================== 构造 & 参数 =====================

/**
 * @brief 构造函数，初始化检测器
 * @param nh ROS节点句柄
 * @param pnh ROS私有节点句柄（用于读取参数）
 */
TargetDetector::TargetDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh),
      tracking_initialized_(false),
      last_target_x_(0.0), last_target_y_(0.0),
      lost_frame_count_(0),
      tcp_server_fd_(-1),
      tcp_port_(5050),
      tcp_server_running_(false),
      heartbeat_(0)
{
    loadParams();

    // 订阅点云话题，使用TCP避免延迟
    lidar_sub_ = nh_.subscribe(lidar_topic_, 1,
                               &TargetDetector::lidarCallback, this,
                               ros::TransportHints().tcpNoDelay());

    // 发布目标位置
    target_pub_ = nh_.advertise<lidar_target_localization::TargetPosition>(output_topic_, 1);

    // 发布调试点云（用于可视化）
    filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/target_filtered_cloud", 1);

    ROS_INFO("[TargetDetector] Started. Listening on: %s", lidar_topic_.c_str());
    ROS_INFO("[TargetDetector] Intensity threshold: %.1f", intensity_threshold_);
    ROS_INFO("[TargetDetector] TCP Port: %d", tcp_port_);

    // 初始化TCP Modbus服务器
    if (initTCPServer()) {
        ROS_INFO("[TargetDetector] TCP server initialized on port %d", tcp_port_);
    } else {
        ROS_WARN("[TargetDetector] Failed to initialize TCP server on port %d", tcp_port_);
    }

    // 启动心跳线程
    heartbeat_thread_ = std::thread(&TargetDetector::heartbeatThread, this);
}

/**
 * @brief 从ROS参数服务器加载配置参数
 */
void TargetDetector::loadParams()
{
    // 输入输出话题
    pnh_.param<std::string>("lidar_topic",    lidar_topic_,   "/velodyne_points");
    pnh_.param<std::string>("output_topic",   output_topic_,  "/target_position");

    // 强度阈值滤波
    pnh_.param("intensity_threshold",  intensity_threshold_,  150.0);

    // 距离和高度ROI范围
    pnh_.param("min_range",            min_range_,            0.5);
    pnh_.param("max_range",            max_range_,            30.0);
    pnh_.param("min_height",           min_height_,           -0.5);
    pnh_.param("max_height",           max_height_,            2.0);

    // 欧式聚类参数
    pnh_.param("cluster_tolerance",    cluster_tolerance_,    0.08);
    pnh_.param("min_cluster_size",     min_cluster_size_,     4);
    pnh_.param("max_cluster_size",     max_cluster_size_,     200);

    // 圆拟合半径范围
    pnh_.param("min_radius",           min_radius_,           0.08);
    pnh_.param("max_radius",           max_radius_,           0.20);

    // 卡尔曼滤波参数
    pnh_.param("use_kalman",           use_kalman_,           true);
    pnh_.param("process_noise_q",      process_noise_q_,      0.01);
    pnh_.param("measurement_noise_r",  measurement_noise_r_,  0.05);

    // 跟踪参数
    pnh_.param("use_tracking",         use_tracking_,         true);
    pnh_.param("tracking_search_radius", tracking_search_radius_, 1.5);
    pnh_.param("max_lost_frames",      max_lost_frames_,      10);

    // TCP端口
    pnh_.param("tcp_port",            tcp_port_,             5050);

    // 配置卡尔曼滤波器噪声参数
    kf_.setProcessNoise(process_noise_q_);
    kf_.setMeasurementNoise(measurement_noise_r_);

    // 初始化Modbus寄存器
    for (auto& r : modbus_registers_) r.store(0, std::memory_order_relaxed);
    // 距离和角度寄存器使用固定值
    modbus_registers_[REG_DISTANCE].store(1000, std::memory_order_relaxed);
    modbus_registers_[REG_ANGLE].store(0, std::memory_order_relaxed);
}

// ===================== 主回调 =====================

/**
 * @brief 激光雷达点云回调函数（核心处理流程）
 * @param msg 输入点云消息
 *
 * 处理流程:
 * 1. 计算帧间时间差（卡尔曼预测）
 * 2. 距离+高度ROI滤波
 * 3. 强度阈值滤波（提取反光点）
 * 4. 跟踪ROI滤波（缩小搜索范围）
 * 5. 欧式聚类
 * 6. 圆拟合+尺寸验证（筛选最佳目标）
 * 7. 卡尔曼滤波平滑
 * 8. 发布ROS消息和TCP数据
 */
void TargetDetector::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Step 0: 计算帧间时间差（供卡尔曼预测用）
    ros::Time current_stamp = msg->header.stamp;
    double dt = 0.1; // 默认10Hz
    if (!last_stamp_.isZero())
        dt = (current_stamp - last_stamp_).toSec();
    if (dt <= 0 || dt > 1.0) dt = 0.1;
    last_stamp_ = current_stamp;

    // Step 1: 转换为PCL点云
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty())
    {
        ROS_WARN_THROTTLE(5.0, "[TargetDetector] Empty point cloud received.");
        return;
    }

    // Step 2: 距离 + 高度 ROI 滤波
    PointCloud::Ptr cloud_roi = rangeHeightFilter(cloud);

    // Step 3: 强度阈值滤波（核心加速步骤）
    // 反光标靶具有高强度反射特性，通过阈值快速筛选
    PointCloud::Ptr cloud_intensity = intensityFilter(cloud_roi);

    if (cloud_intensity->empty())
    {
        ROS_DEBUG_THROTTLE(2.0, "[TargetDetector] No high-intensity points.");
        // 卡尔曼预测保持输出（预测位置）
        if (use_kalman_ && kf_.isInitialized())
        {
            kf_.predict(dt);
            lost_frame_count_++;
        }
        if (lost_frame_count_ > max_lost_frames_)
        {
            tracking_initialized_ = false;
            ROS_WARN_THROTTLE(3.0, "[TargetDetector] Target lost, reset tracking.");
        }
        return;
    }

    // Step 4: 跟踪ROI（上一帧位置附近缩小搜索范围）
    // 如果已初始化跟踪，只在上一帧目标附近搜索，提高速度和鲁棒性
    PointCloud::Ptr cloud_search = cloud_intensity;
    if (use_tracking_ && tracking_initialized_)
        cloud_search = trackingROIFilter(cloud_intensity);

    if (cloud_search->empty())
        cloud_search = cloud_intensity; // 降级：用全部高强度点

    // Step 5: 欧式聚类
    // 将高强度点分成多个簇，每个簇可能是一个标靶
    std::vector<pcl::PointIndices> clusters = euclideanClustering(cloud_search);

    if (clusters.empty())
    {
        ROS_DEBUG_THROTTLE(2.0, "[TargetDetector] No clusters found.");
        lost_frame_count_++;
        if (lost_frame_count_ > max_lost_frames_)
            tracking_initialized_ = false;
        return;
    }

    // Step 6: 遍历候选簇，进行圆拟合 + 尺寸验证，选最佳簇
    double best_cx = 0, best_cy = 0, best_radius = 0;
    int    best_count = 0;
    bool   found = false;
    double best_score = 1e9; // 拟合残差越小越好

    for (const auto& cluster : clusters)
    {
        double cx, cy, radius;
        if (!fitCircle(cloud_search, cluster, cx, cy, radius))
            continue;

        // 尺寸验证：标靶半径应在指定范围内
        if (radius < min_radius_ || radius > max_radius_)
            continue;

        // 如果有跟踪历史，优先选距离上一帧最近的（目标连续性）
        // 否则，选择点数最多的（反光最明显）
        double score;
        if (tracking_initialized_)
        {
            double dx = cx - last_target_x_;
            double dy = cy - last_target_y_;
            score = std::sqrt(dx*dx + dy*dy);
        }
        else
        {
            score = -static_cast<double>(cluster.indices.size()); // 点数多优先
        }

        if (!found || score < best_score)
        {
            best_score  = score;
            best_cx     = cx;
            best_cy     = cy;
            best_radius = radius;
            best_count  = static_cast<int>(cluster.indices.size());
            found       = true;
        }
    }

    if (!found)
    {
        lost_frame_count_++;
        if (lost_frame_count_ > max_lost_frames_)
            tracking_initialized_ = false;
        return;
    }

    // Step 7: 卡尔曼滤波
    // 使用卡尔曼滤波平滑检测结果，减少噪声影响
    double filtered_x = best_cx;
    double filtered_y = best_cy;

    if (use_kalman_)
    {
        kf_.predict(dt);
        Eigen::Vector2d est = kf_.update(best_cx, best_cy);
        filtered_x = est(0);
        filtered_y = est(1);
    }

    // 更新跟踪状态
    tracking_initialized_ = true;
    last_target_x_ = filtered_x;
    last_target_y_ = filtered_y;
    lost_frame_count_ = 0;

    // Step 8: 发布结果
    // 激光雷达坐标系以自身为原点:
    //   标靶在激光雷达坐标系: (filtered_x, filtered_y)
    //   小车在标靶坐标系:    (-filtered_x, -filtered_y)
    double distance = std::sqrt(filtered_x*filtered_x + filtered_y*filtered_y);

    lidar_target_localization::TargetPosition out_msg;
    out_msg.header.stamp    = current_stamp;
    out_msg.header.frame_id = msg->header.frame_id;
    out_msg.x           = -filtered_x;   // 小车相对标靶 X
    out_msg.y           = -filtered_y;   // 小车相对标靶 Y
    out_msg.distance    = distance;
    out_msg.target_x    = filtered_x;    // 标靶在激光雷达系 X
    out_msg.target_y    = filtered_y;    // 标靶在激光雷达系 Y
    out_msg.point_count = best_count;
    out_msg.is_valid    = true;

    target_pub_.publish(out_msg);

    // 发布TCP数据：雷达相对于小车的坐标 (-filtered_x, -filtered_y)
    sendTCPData(-filtered_x, -filtered_y);

    ROS_DEBUG("[TargetDetector] Target at lidar frame: (%.3f, %.3f) m, r=%.3f m, pts=%d",
              filtered_x, filtered_y, best_radius, best_count);

    // 调试：发布高强度点云（可在rviz观察）
    if (filtered_cloud_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 debug_cloud;
        pcl::toROSMsg(*cloud_intensity, debug_cloud);
        debug_cloud.header = msg->header;
        filtered_cloud_pub_.publish(debug_cloud);
    }
}

// ===================== 滤波函数 =====================

/**
 * @brief 距离和高度ROI滤波
 * @param cloud 输入点云
 * @return 滤波后的点云
 *
 * 滤除超出指定距离范围和高度范围的点，减少计算量
 */
PointCloud::Ptr TargetDetector::rangeHeightFilter(const PointCloud::Ptr& cloud)
{
    PointCloud::Ptr result(new PointCloud);
    result->reserve(cloud->size());

    for (const auto& pt : cloud->points)
    {
        float range = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        if (range < min_range_ || range > max_range_) continue;
        if (pt.z < min_height_ || pt.z > max_height_) continue;
        result->points.push_back(pt);
    }
    result->width  = result->points.size();
    result->height = 1;
    result->is_dense = false;
    return result;
}

/**
 * @brief 强度阈值滤波（核心步骤）
 * @param cloud 输入点云
 * @return 滤波后的点云
 *
 * 提取强度大于阈值的高反射点。
 * 反光标靶具有高反射特性，强度显著高于普通物体。
 * 此步骤可大幅减少后续处理的数据量。
 */
PointCloud::Ptr TargetDetector::intensityFilter(const PointCloud::Ptr& cloud)
{
    PointCloud::Ptr result(new PointCloud);
    result->reserve(64); // 高强度点通常很少，预分配小内存

    for (const auto& pt : cloud->points)
    {
        if (pt.intensity >= intensity_threshold_)
            result->points.push_back(pt);
    }
    result->width  = result->points.size();
    result->height = 1;
    result->is_dense = false;
    return result;
}

/**
 * @brief 跟踪ROI滤波（缩小搜索范围）
 * @param cloud 输入点云
 * @return 滤波后的点云
 *
 * 只保留上一帧目标位置附近指定半径内的点。
 * 提高目标连续性，减少误检和计算量。
 */
PointCloud::Ptr TargetDetector::trackingROIFilter(const PointCloud::Ptr& cloud)
{
    PointCloud::Ptr result(new PointCloud);
    double r2 = tracking_search_radius_ * tracking_search_radius_;

    for (const auto& pt : cloud->points)
    {
        double dx = pt.x - last_target_x_;
        double dy = pt.y - last_target_y_;
        if (dx*dx + dy*dy <= r2)
            result->points.push_back(pt);
    }
    result->width  = result->points.size();
    result->height = 1;
    result->is_dense = false;
    return result;
}

// ===================== 聚类 =====================

/**
 * @brief 欧式聚类
 * @param cloud 输入点云
 * @return 聚类索引列表
 *
 * 使用KD树加速的欧式聚类算法。
 * 将邻近的点分为同一簇，每个簇可能对应一个标靶。
 */
std::vector<pcl::PointIndices> TargetDetector::euclideanClustering(const PointCloud::Ptr& cloud)
{
    std::vector<pcl::PointIndices> cluster_indices;
    if (cloud->empty()) return cluster_indices;

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);    // 聚类距离容差
    ec.setMinClusterSize(min_cluster_size_);      // 最小点数
    ec.setMaxClusterSize(max_cluster_size_);      // 最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    return cluster_indices;
}

// ===================== 圆拟合（代数法，速度最快）=====================

/**
 * @brief 最小二乘法圆拟合
 * @param cloud 输入点云
 * @param indices 簇索引
 * @param cx 输出圆心X坐标
 * @param cy 输出圆心Y坐标
 * @param radius 输出半径
 * @return 成功返回true，失败返回false
 *
 * 使用代数最小二乘法拟合2D圆心。
 * 算法速度快，但对异常点敏感，需要配合强度滤波和聚类使用。
 *
 * 数学推导:
 *   圆方程: (x-cx)^2 + (y-cy)^2 = r^2
 *   展开: x^2 + y^2 = 2*cx*x + 2*cy*y + (r^2 - cx^2 - cy^2)
 *   令 A=2cx, B=2cy, C=r^2-cx^2-cy^2
 *   线性方程: A*xi + B*yi + C = xi^2 + yi^2
 *   最小二乘解: [A,B,C] = (M^T M)^-1 M^T b
 */
bool TargetDetector::fitCircle(const PointCloud::Ptr& cloud,
                                const pcl::PointIndices& indices,
                                double& cx, double& cy, double& radius)
{
    int n = static_cast<int>(indices.indices.size());
    if (n < 3) return false;

    // 代数法最小二乘圆拟合
    // 方程：(x-cx)^2 + (y-cy)^2 = r^2
    // 展开：x^2 + y^2 = 2*cx*x + 2*cy*y + (r^2 - cx^2 - cy^2)
    // 令 A=2cx, B=2cy, C=r^2-cx^2-cy^2
    // 则：A*xi + B*yi + C = xi^2 + yi^2
    // 用最小二乘解：[A,B,C] = (M^T M)^-1 M^T b

    Eigen::MatrixXd M(n, 3);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i)
    {
        const auto& pt = cloud->points[indices.indices[i]];
        double x = pt.x, y = pt.y;
        M(i, 0) = x;
        M(i, 1) = y;
        M(i, 2) = 1.0;
        b(i)    = x*x + y*y;
    }

    // 正规方程求解（使用LDLT分解，数值稳定性好）
    Eigen::Vector3d params = (M.transpose() * M).ldlt().solve(M.transpose() * b);

    cx     = params(0) / 2.0;
    cy     = params(1) / 2.0;
    radius = std::sqrt(params(2) + cx*cx + cy*cy);

    // 验证拟合质量（计算点到圆的均方误差）
    double mse = 0.0;
    for (int i = 0; i < n; ++i)
    {
        const auto& pt = cloud->points[indices.indices[i]];
        double dx = pt.x - cx, dy = pt.y - cy;
        double err = std::abs(std::sqrt(dx*dx + dy*dy) - radius);
        mse += err * err;
    }
    mse /= n;

    // 拟合误差过大则拒绝（阈值：5cm）
    if (mse > 0.0025) // 0.05^2
    {
        ROS_DEBUG("[TargetDetector] Circle fit MSE too large: %.4f", mse);
        return false;
    }

    if (std::isnan(cx) || std::isnan(cy) || std::isnan(radius))
        return false;

    return true;
}

// ===================== TCP服务器 =====================

/**
 * @brief 初始化TCP Modbus服务器
 * @return 成功返回true，失败返回false
 *
 * 创建非阻塞TCP socket，绑定指定端口，开始监听。
 * 使用select实现多客户端并发处理。
 */
bool TargetDetector::initTCPServer()
{
    tcp_server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_server_fd_ < 0) {
        ROS_ERROR("[TargetDetector] Failed to create socket: %s", strerror(errno));
        return false;
    }

    int opt = 1;
    setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    // 配置socket选项
    setSockBuf(tcp_server_fd_, 512 * 1024, 512 * 1024);  // 512KB缓冲区
    setNoDelay(tcp_server_fd_);                           // 禁用Nagle算法
    setNonBlocking(tcp_server_fd_);                        // 非阻塞模式

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(tcp_port_);

    if (bind(tcp_server_fd_, (struct sockaddr*)&server_addr,
             sizeof(server_addr)) < 0) {
        ROS_ERROR("[TargetDetector] Failed to bind to port %d: %s", tcp_port_, strerror(errno));
        close(tcp_server_fd_);
        tcp_server_fd_ = -1;
        return false;
    }

    if (listen(tcp_server_fd_, 64) < 0) {
        ROS_ERROR("[TargetDetector] Failed to listen on socket: %s", strerror(errno));
        close(tcp_server_fd_);
        tcp_server_fd_ = -1;
        return false;
    }

    tcp_server_running_.store(true, std::memory_order_release);
    tcp_server_thread_ = std::thread(&TargetDetector::tcpServerThread, this);
    return true;
}

/**
 * @brief 关闭TCP服务器
 */
void TargetDetector::closeTCPServer()
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

/**
 * @brief 移除TCP客户端
 * @param client_fd 客户端socket文件描述符
 */
void TargetDetector::removeClient(int client_fd)
{
    auto it = std::find(tcp_client_fds_.begin(), tcp_client_fds_.end(), client_fd);
    if (it != tcp_client_fds_.end()) tcp_client_fds_.erase(it);
    client_rx_caches_.erase(client_fd);

    shutdown(client_fd, SHUT_RDWR);
    close(client_fd);

    ROS_WARN("[TargetDetector] TCP client removed (fd=%d), total clients: %zu", client_fd,
             tcp_client_fds_.size());
}

/**
 * @brief TCP服务器线程
 *
 * 使用select实现多路复用，同时处理监听和多个客户端连接。
 * 支持Modbus TCP协议解析。
 */
void TargetDetector::tcpServerThread()
{
    ROS_INFO("[TargetDetector] TCP server thread started, port=%d", tcp_port_);

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

        // 200ms超时，定期检查退出标志
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200 * 1000;

        int ret = select(max_fd + 1, &read_fds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (!tcp_server_running_.load()) break;
            if (errno == EINTR) continue;
            ROS_WARN("[TargetDetector] select error: %s", strerror(errno));
            continue;
        }
        if (!tcp_server_running_.load()) break;

        // 处理新连接
        if (tcp_server_fd_ >= 0 && FD_ISSET(tcp_server_fd_, &read_fds)) {
            while (true) {
                struct sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
                int client_fd = accept(tcp_server_fd_, (struct sockaddr*)&client_addr,
                                      &client_len);
                if (client_fd < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    if (errno == EINTR) continue;
                    ROS_WARN("[TargetDetector] accept error: %s", strerror(errno));
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
                ROS_INFO("[TargetDetector] TCP client connected: %s:%d (fd=%d), total=%zu", ip,
                         ntohs(client_addr.sin_port), client_fd,
                         tcp_client_fds_.size());
            }
        }

        // 处理客户端数据
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

            // 解析Modbus TCP帧
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

    ROS_INFO("[TargetDetector] TCP server thread stopped");
}

/**
 * @brief 心跳线程
 *
 * 每20ms更新一次心跳寄存器，用于检测连接状态。
 */
void TargetDetector::heartbeatThread()
{
    ROS_INFO("[TargetDetector] Heartbeat thread started (20ms)");
    while (tcp_server_running_.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        uint16_t hb = heartbeat_.fetch_add(1) + 1;
        if (hb == 0) {
            heartbeat_.store(1);
            hb = 1;
        }
        modbus_registers_[REG_HEARTBEAT].store(hb, std::memory_order_relaxed);
    }
    ROS_INFO("[TargetDetector] Heartbeat thread stopped");
}

/**
 * @brief 发送TCP数据（更新Modbus寄存器）
 * @param lidar_x 雷达相对小车的X坐标（米）
 * @param lidar_y 雷达相对小车的Y坐标（米）
 *
 * 将坐标转换为毫米存储到Modbus寄存器。
 * 坐标限制在int16范围内(-32768~32767)。
 * 距离和角度使用固定值（1000mm和0度）。
 */
void TargetDetector::sendTCPData(double lidar_x, double lidar_y)
{
  int32_t x_mm32 = static_cast<int32_t>(std::llround(lidar_x * 1000.0));
  int32_t y_mm32 = static_cast<int32_t>(std::llround(lidar_y * 1000.0));

  // 限制在int16范围内
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

/**
 * @brief 处理Modbus TCP请求
 * @param client_fd 客户端socket
 * @param request 请求数据
 * @param length 请求数据长度
 *
 * 支持的功能码:
 * - 0x03: 读保持寄存器
 * - 0x06: 写单个保持寄存器
 */
void TargetDetector::handleModbusRequest(int client_fd, const uint8_t* request, ssize_t length)
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

    // 复制头部
    response[0] = request[0];
    response[1] = request[1];
    response[2] = request[2];
    response[3] = request[3];
    response[6] = unit_id;

    switch (function_code) {
        case 0x03: {
            // 读保持寄存器
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
            // 写单个保持寄存器
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
            // 不支持的功能码
            response[7] = function_code | 0x80;
            response[8] = 0x01;
            response_len = 9;
            break;
        }
    }

    // 更新长度字段
    response[4] = static_cast<uint8_t>(((response_len - 6) >> 8) & 0xFF);
    response[5] = static_cast<uint8_t>((response_len - 6) & 0xFF);

    // 发送响应（100ms超时）
    const int SEND_DEADLINE_MS = 100;
    bool ok = sendAllWithDeadlineMs(client_fd, response,
                                    static_cast<size_t>(response_len),
                                    SEND_DEADLINE_MS);
    if (!ok) {
        ROS_WARN("[TargetDetector] send Modbus response failed/timeout (fd=%d, errno=%s). drop client.",
                 client_fd, strerror(errno));
        removeClient(client_fd);
    }
}

/**
 * @brief 主函数
 * @param argc 参数个数
 * @param argv 参数列表
 * @return 程序退出码
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_detector");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    TargetDetector detector(nh, pnh);

    // 多线程spinner保证实时回调不阻塞
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
