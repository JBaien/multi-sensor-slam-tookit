#include "lidar_target_detection/lidar_target_detector.h"

#include <geometry_msgs/PointStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigenvalues>
#include <iomanip>
#include <sstream>

// ====== TCP/Modbus 相关修复所需(仅 TCP 层使用)======
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>

namespace {
static bool setNonBlocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return false;
    return true;
}

static bool sendAll(int fd, const uint8_t* data, size_t len)
{
    size_t sent_total = 0;
    int eagain_tries = 0;
    while (sent_total < len) {
        ssize_t n = send(fd, data + sent_total, len - sent_total, 0);
        if (n > 0) {
            sent_total += static_cast<size_t>(n);
            eagain_tries = 0;
            continue;
        }
        if (n < 0 && (errno == EINTR)) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // 非阻塞短暂不可写:认为连接异常,直接失败让上层关闭
            // 修复:等待可写后继续发送,避免误断链导致"几秒才更新"
            if (++eagain_tries > 20) { // 最多等待约 1 秒
                return false;
            }
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 50 * 1000; // 50ms
            int r = select(fd + 1, nullptr, &wfds, nullptr, &tv);
            if (r > 0) {
                continue;
            }
            if (r < 0 && errno == EINTR) {
                continue;
            }
            continue;
        }
        return false;
    }
    return true;
}
} // namespace

/**
 * @brief 构造函数,初始化激光雷达反光标靶检测器
 * @param nh ROS节点句柄
 * @param private_nh 私有节点句柄,用于获取参数
 *
 * 功能:初始化所有参数、订阅者、发布者和卡尔曼滤波器
 */
LidarTargetDetector::LidarTargetDetector(ros::NodeHandle& nh,
                                         ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      tcp_server_fd_(-1),
      tcp_client_fd_(-1),
      tcp_port_(5050),
      tcp_server_running_(false),
      heartbeat_(0)
{
    // 从参数服务器获取算法参数
    private_nh_.param("intensity_threshold", intensity_threshold_,
                      100.0); // 反射强度阈值
    private_nh_.param("cluster_tolerance", cluster_tolerance_,
                      0.1); // 聚类距离容差
    private_nh_.param("min_cluster_size", min_cluster_size_,
                      10); // 最小聚类点数
    private_nh_.param("max_cluster_size", max_cluster_size_,
                      500); // 最大聚类点数
    private_nh_.param("min_arc_angle", min_arc_angle_,
                      30.0); // 最小圆弧角度(度)
    private_nh_.param("max_arc_angle", max_arc_angle_,
                      180.0); // 最大圆弧角度(度)
    private_nh_.param("max_rmse_threshold", max_rmse_threshold_,
                      0.05); // 最大RMSE阈值
    private_nh_.param("min_target_radius", min_target_radius_,
                      0.05); // 最小目标半径(m)
    private_nh_.param("max_target_radius", max_target_radius_,
                      0.3); // 最大目标半径(m)

    // 初始化ROS订阅者和发布者
    cloud_sub_ = nh_.subscribe("input_cloud", 1,
                               &LidarTargetDetector::cloudCallback, this);
    target_pub_ =
        nh_.advertise<geometry_msgs::PointStamped>("target_position", 1);
    filtered_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("clusters", 1);

    // 初始化卡尔曼滤波器状态
    state_ = Eigen::Vector4d::Zero(); // [x, y, vx, vy] - 位置和速度状态
    P_ = Eigen::Matrix4d::Identity() * 10.0;  // 初始状态协方差矩阵
    H_ = Eigen::Matrix<double, 2, 4>::Zero(); // 观测矩阵
    H_(0, 0) = 1.0;                           // 观测x位置
    H_(1, 1) = 1.0;                           // 观测y位置

    // 过程噪声(假设目标移动较慢)
    Q_ = Eigen::Matrix4d::Identity() * 0.01;

    // 观测噪声(基于激光雷达测量精度)
    R_ = Eigen::Matrix2d::Identity() * 0.001;

    // 初始化Modbus寄存器
    memset(modbus_registers_, 0, sizeof(modbus_registers_));

    // 初始化固定值寄存器
    modbus_registers_[REG_DISTANCE] = 1000; // 移动距离: 1000mm
    modbus_registers_[REG_ANGLE] = 0;       // 移动角度: 0

    // 输出初始化信息
    ROS_INFO("Lidar Target Detector initialized with parameters:");
    ROS_INFO("  Intensity threshold: %.1f", intensity_threshold_);
    ROS_INFO("  Cluster tolerance: %.3f m", cluster_tolerance_);
    ROS_INFO("  Min/Max cluster size: %d/%d", min_cluster_size_,
             max_cluster_size_);
    ROS_INFO("  Arc angle range: %.1f-%.1f degrees", min_arc_angle_,
             max_arc_angle_);
    ROS_INFO("  Max RMSE threshold: %.3f", max_rmse_threshold_);
    ROS_INFO("  Target radius range: %.3f-%.3f m", min_target_radius_,
             max_target_radius_);

    // 初始化TCP服务器
    if (initTCPServer()) {
        ROS_INFO("TCP server initialized on port %d", tcp_port_);
    } else {
        ROS_WARN("Failed to initialize TCP server on port %d", tcp_port_);
    }

    // 启动心跳线程
    heartbeat_thread_ =
        std::thread(&LidarTargetDetector::heartbeatThread, this);
}

/**
 * @brief 激光雷达点云回调函数,处理接收到的点云数据
 * @param cloud_msg 接收到的激光雷达点云消息
 *
 * 功能流程:
 * 1. 转换点云格式
 * 2. 根据反射强度过滤点云
 * 3. 对过滤后的点云进行聚类
 * 4. 选择反射强度最高的聚类进行圆弧拟合
 * 5. 使用卡尔曼滤波器平滑位置估计
 * 6. 发布检测结果
 */
void LidarTargetDetector::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // 步骤1: 转换ROS点云消息为PCL格式
    pcl::PointCloud<PointXYZIRT>::Ptr cloud(new pcl::PointCloud<PointXYZIRT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (cloud->empty()) {
        ROS_WARN("Received empty point cloud");
        return;
    }

    ROS_DEBUG("Received point cloud with %zu points", cloud->size());

    // 步骤2: 根据反射强度过滤点云,提取反光标靶数据
    pcl::PointCloud<PointXYZIRT>::Ptr filtered_cloud = filterByIntensity(cloud);

    if (filtered_cloud->empty()) {
        ROS_WARN("No points above intensity threshold %.1f",
                 intensity_threshold_);
        // 调试:检查原始点云的强度分布
        float max_intensity = 0.0;
        for (const auto& point : cloud->points) {
            if (point.intensity > max_intensity) {
                max_intensity = point.intensity;
            }
        }
        ROS_WARN("Maximum intensity in raw cloud: %.1f", max_intensity);
        return;
    }

    ROS_INFO("Filtered cloud has %zu points above intensity threshold",
             filtered_cloud->size());

    // 步骤3: 对高反射强度点云进行聚类,分离不同的反光标靶
    std::vector<pcl::PointCloud<PointXYZIRT>::Ptr> clusters =
        extractClusters(filtered_cloud);

    if (clusters.empty()) {
        ROS_WARN("No clusters found in filtered cloud");
        return;
    }

    ROS_INFO("Found %zu clusters", clusters.size());

    // 调试:输出每个聚类的大小
    for (size_t i = 0; i < clusters.size(); ++i) {
        ROS_INFO("Cluster %zu: %zu points", i, clusters[i]->size());
    }

    // 步骤4: 选择反射强度最高的聚类进行圆弧拟合
    pcl::PointCloud<PointXYZIRT>::Ptr highest_intensity_cluster = nullptr;
    double max_avg_intensity = 0.0;

    // 遍历所有聚类,计算每个聚类的平均反射强度
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        if (cluster->size() >= min_cluster_size_) {
            double avg_intensity = 0.0;
            for (const auto& point : cluster->points) {
                avg_intensity += point.intensity;
            }
            avg_intensity /= cluster->size();

            ROS_INFO("Cluster %zu: size=%zu, avg_intensity=%.1f", i,
                     cluster->size(), avg_intensity);

            // 选择平均反射强度最高的聚类
            if (avg_intensity > max_avg_intensity) {
                max_avg_intensity = avg_intensity;
                highest_intensity_cluster = cluster;
            }
        } else {
            ROS_INFO("Cluster %zu: size=%zu (too small, min required=%d)", i,
                     cluster->size(), min_cluster_size_);
        }
    }

    // 步骤5: 只对反射强度最高的聚类进行圆弧拟合
    geometry_msgs::PointStamped best_target;
    bool target_found = false;

    if (highest_intensity_cluster != nullptr) {
        ROS_DEBUG(
            "Processing highest intensity cluster with avg intensity: %.1f",
            max_avg_intensity);

        // 对选定的聚类进行圆弧拟合,获取标靶中心位置
        if (fitArcAndGetCenter(highest_intensity_cluster, best_target)) {
            target_found = true;
            ROS_DEBUG("Target detected from highest intensity cluster");
        }
    }

    // 5. 如果找到目标,更新卡尔曼滤波器并发布
    if (target_found) {
        updateKalmanFilter(best_target);

        // 发布滤波后的位置
        geometry_msgs::PointStamped filtered_position;
        filtered_position.header = cloud_msg->header;
        filtered_position.point.x = state_(0);
        filtered_position.point.y = state_(1);
        filtered_position.point.z = 0.0;

        target_pub_.publish(filtered_position);

        // 通过TCP发送数据
        sendTCPData(filtered_position);

        ROS_DEBUG(
            "Target detected at (%.3f, %.3f) from highest intensity cluster "
            "(avg: %.1f)",
            filtered_position.point.x, filtered_position.point.y,
            max_avg_intensity);
    }

    // 发布过滤后的点云用于可视化(转换为单色点云)
    pcl::PointCloud<pcl::PointXYZRGB> single_color_cloud;
    for (const auto& point : filtered_cloud->points) {
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        colored_point.r = 255; // 红色
        colored_point.g = 0;
        colored_point.b = 0;
        single_color_cloud.push_back(colored_point);
    }

    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(single_color_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = cloud_msg->header;
    filtered_cloud_pub_.publish(filtered_cloud_msg);

    // 发布聚类可视化点云(不同聚类用不同颜色)
    if (!clusters.empty()) {
        publishClustersVisualization(clusters, cloud_msg->header);
    }
}

/**
 * @brief 根据反射强度过滤点云,提取反光标靶数据
 * @param cloud 输入点云
 * @return 过滤后的高反射强度点云
 *
 * 功能:遍历点云中的所有点,只保留反射强度高于阈值的点
 * 这些点通常对应于反光标靶的表面
 */
pcl::PointCloud<PointXYZIRT>::Ptr LidarTargetDetector::filterByIntensity(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    pcl::PointCloud<PointXYZIRT>::Ptr filtered_cloud(
        new pcl::PointCloud<PointXYZIRT>);

    // 遍历所有点,筛选反射强度高于阈值的点
    for (const auto& point : cloud->points) {
        if (point.intensity >= intensity_threshold_) {
            filtered_cloud->push_back(point);
        }
    }

    // 设置点云属性
    filtered_cloud->width = filtered_cloud->size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    return filtered_cloud;
}

std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>
LidarTargetDetector::extractClusters(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud)
{
    std::vector<pcl::PointCloud<PointXYZIRT>::Ptr> clusters;

    // 将自定义点云转换为标准PointXYZI点云进行聚类
    pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& point : cloud->points) {
        pcl::PointXYZI xyz_point;
        xyz_point.x = point.x;
        xyz_point.y = point.y;
        xyz_point.z = point.z;
        xyz_point.intensity = point.intensity;
        xyz_cloud->push_back(xyz_point);
    }

    // 创建KD树用于聚类(使用标准点类型)
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(xyz_cloud);

    // 欧几里得聚类(使用标准点类型)
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(xyz_cloud);
    ec.extract(cluster_indices);

    // 提取聚类点云(转换回自定义点类型)
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<PointXYZIRT>::Ptr cluster(
            new pcl::PointCloud<PointXYZIRT>);
        for (const auto& idx : indices.indices) {
            cluster->push_back((*cloud)[idx]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }

    return clusters;
}

bool LidarTargetDetector::fitArcAndGetCenter(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cluster,
    geometry_msgs::PointStamped& center)
{
    if (cluster->size() < 10) {
        return false;
    }

    // 计算点云在Z轴上的平均高度,作为圆柱体的高度参考
    double sum_z = 0.0;
    for (const auto& point : cluster->points) {
        sum_z += point.z;
    }
    double avg_z = sum_z / cluster->size();

    // 使用最小二乘法在XY平面拟合圆弧
    double center_x, center_y, radius, rmse;

    // 构建最小二乘拟合矩阵
    int n = cluster->size();
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);

    // 计算质心
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

    // 求解最小二乘问题
    Eigen::Vector3d solution =
        A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // 计算圆心和半径
    center_x = solution(0) + mean_x;
    center_y = solution(1) + mean_y;
    radius = std::sqrt(solution(0) * solution(0) + solution(1) * solution(1) +
                       solution(2));

    // 检查半径是否在合理范围内
    if (radius < min_target_radius_ || radius > max_target_radius_) {
        return false;
    }

    // 计算RMSE(拟合质量)
    double sum_sq_error = 0.0;
    for (int i = 0; i < n; ++i) {
        double dx = cluster->points[i].x - center_x;
        double dy = cluster->points[i].y - center_y;
        double actual_radius = std::sqrt(dx * dx + dy * dy);
        sum_sq_error += (actual_radius - radius) * (actual_radius - radius);
    }
    rmse = std::sqrt(sum_sq_error / n);

    // 检查拟合质量
    if (rmse > max_rmse_threshold_) {
        return false;
    }

    // 计算圆弧角度范围(验证是否为部分圆柱体)
    std::vector<double> angles;
    for (const auto& p : cluster->points) {
        double dx = p.x - center_x;
        double dy = p.y - center_y;
        double angle = std::atan2(dy, dx);
        angles.push_back(angle);
    }

    // 排序角度并计算最大间隔
    std::sort(angles.begin(), angles.end());
    double max_gap = 0.0;
    for (size_t i = 0; i < angles.size(); ++i) {
        double gap = angles[(i + 1) % angles.size()] - angles[i];
        if (gap < 0) gap += 2 * M_PI;
        max_gap = std::max(max_gap, gap);
    }

    double arc_angle = 2 * M_PI - max_gap;
    double arc_angle_deg = arc_angle * 180.0 / M_PI;

    // 检查圆弧角度是否在合理范围内
    if (arc_angle_deg < min_arc_angle_ || arc_angle_deg > max_arc_angle_) {
        return false;
    }

    // 设置中心点坐标(使用平均高度)
    center.point.x = center_x;
    center.point.y = center_y;
    center.point.z = avg_z;

    ROS_DEBUG(
        "Arc fitted: center=(%.3f, %.3f, %.3f), radius=%.3f, angle=%.1f°, "
        "RMSE=%.4f",
        center_x, center_y, avg_z, radius, arc_angle_deg, rmse);

    return true;
}

void LidarTargetDetector::updateKalmanFilter(
    const geometry_msgs::PointStamped& measurement)
{
    // 预测步骤
    double dt = 0.1;   // 假设时间间隔为0.1秒
    Eigen::Matrix4d F; // 状态转移矩阵
    F << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

    state_ = F * state_;
    P_ = F * P_ * F.transpose() + Q_;

    // 更新步骤
    Eigen::Vector2d z(measurement.point.x, measurement.point.y); // 观测值
    Eigen::Vector2d y = z - H_ * state_;                         // 观测残差
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_; // 残差协方差
    Eigen::Matrix<double, 4, 2> K =
        P_ * H_.transpose() * S.inverse(); // 卡尔曼增益

    state_ = state_ + K * y;
    P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
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

/**
 * @brief 发布聚类可视化点云,不同聚类用不同颜色
 * @param clusters 聚类列表
 * @param header 点云消息头
 */
void LidarTargetDetector::publishClustersVisualization(
    const std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>& clusters,
    const std_msgs::Header& header)
{
    // 创建包含所有聚类的彩色点云
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;

    // 预定义颜色列表(RGB格式)
    std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> colors = {
        {255, 0, 0},   // 红色
        {0, 255, 0},   // 绿色
        {0, 0, 255},   // 蓝色
        {255, 255, 0}, // 黄色
        {255, 0, 255}, // 洋红色
        {0, 255, 255}, // 青色
        {255, 128, 0}, // 橙色
        {128, 0, 255}, // 紫色
        {128, 255, 0}, // 黄绿色
        {0, 128, 255}  // 天蓝色
    };

    // 为每个聚类分配颜色并添加到彩色点云中
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        auto color = colors[i % colors.size()]; // 循环使用颜色

        for (const auto& point : cluster->points) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = std::get<0>(color);
            colored_point.g = std::get<1>(color);
            colored_point.b = std::get<2>(color);
            colored_cloud.push_back(colored_point);
        }
    }

    // 发布彩色点云
    if (!colored_cloud.empty()) {
        sensor_msgs::PointCloud2 clusters_msg;
        pcl::toROSMsg(colored_cloud, clusters_msg);
        clusters_msg.header = header;
        clusters_pub_.publish(clusters_msg);
    }
}

/**
 * @brief 心跳线程,每20ms递增心跳计数器
 */
void LidarTargetDetector::heartbeatThread()
{
    ROS_INFO("Heartbeat thread started, updating every 20ms");
    while (tcp_server_running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        heartbeat_++;
        if (heartbeat_ == 0) { // 防止溢出
            heartbeat_ = 1;
        }
        {
            std::lock_guard<std::mutex> lock(tcp_mutex_);
            modbus_registers_[REG_HEARTBEAT] = heartbeat_.load();
        }
    }
    ROS_INFO("Heartbeat thread stopped");
}

/**
 * @brief 初始化TCP服务器
 * @return 初始化成功返回true,失败返回false
 */
bool LidarTargetDetector::initTCPServer()
{
    // 创建socket
    tcp_server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_server_fd_ < 0) {
        ROS_ERROR("Failed to create socket: %s", strerror(errno));
        return false;
    }

    // 设置socket选项,允许地址重用
    int opt = 1;
    if (setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt,
                   sizeof(opt)) < 0) {
        ROS_ERROR("Failed to set socket options: %s", strerror(errno));
        close(tcp_server_fd_);
        return false;
    }

    // 绑定地址和端口
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(tcp_port_);

    if (bind(tcp_server_fd_, (struct sockaddr*)&server_addr,
             sizeof(server_addr)) < 0) {
        ROS_ERROR("Failed to bind to port %d: %s", tcp_port_, strerror(errno));
        close(tcp_server_fd_);
        return false;
    }

    // 开始监听
    if (listen(tcp_server_fd_, 1) < 0) {
        ROS_ERROR("Failed to listen on socket: %s", strerror(errno));
        close(tcp_server_fd_);
        return false;
    }

    // 使 listen socket 非阻塞(仅 TCP 层)
    setNonBlocking(tcp_server_fd_);

    // 启动服务器线程
    tcp_server_running_ = true;
    tcp_server_thread_ =
        std::thread(&LidarTargetDetector::tcpServerThread, this);

    return true;
}

/**
 * @brief 关闭TCP服务器
 */
void LidarTargetDetector::closeTCPServer()
{
    tcp_server_running_ = false;

    {
        std::lock_guard<std::mutex> lock(tcp_mutex_);
        if (tcp_client_fd_ >= 0) {
            shutdown(tcp_client_fd_, SHUT_RDWR);
            close(tcp_client_fd_);
            tcp_client_fd_ = -1;
        }
    }

    if (tcp_server_fd_ >= 0) {
        shutdown(tcp_server_fd_, SHUT_RDWR);
        close(tcp_server_fd_);
        tcp_server_fd_ = -1;
    }

    if (tcp_server_thread_.joinable()) {
        tcp_server_thread_.join();
    }
}

/**
 * @brief TCP服务器线程,处理Modbus TCP请求
 */
void LidarTargetDetector::tcpServerThread()
{
    ROS_INFO("TCP server thread started, waiting for connections on port %d",
             tcp_port_);

    // Modbus TCP 粘包/半包处理缓存(仅 TCP 层)
    std::vector<uint8_t> rx_cache;
    rx_cache.reserve(1024);

    while (tcp_server_running_) {
        // 设置socket为非阻塞模式,以便能够检查tcp_server_running_
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(tcp_server_fd_, &read_fds);
        int max_fd = tcp_server_fd_;

        int client_fd_snapshot = -1;
        {
            std::lock_guard<std::mutex> lock(tcp_mutex_);
            client_fd_snapshot = tcp_client_fd_;
        }

        if (client_fd_snapshot >= 0) {
            FD_SET(client_fd_snapshot, &read_fds);
            if (client_fd_snapshot > max_fd) max_fd = client_fd_snapshot;
        }

        int ret = select(max_fd + 1, &read_fds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (tcp_server_running_) {
                ROS_ERROR("select error: %s", strerror(errno));
            }
            continue;
        }

        if (!tcp_server_running_) {
            break;
        }

        if (FD_ISSET(tcp_server_fd_, &read_fds)) {
            // 接受客户端连接
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(tcp_server_fd_,
                                   (struct sockaddr*)&client_addr, &client_len);

            if (client_fd < 0) {
                if (tcp_server_running_) {
                    ROS_WARN("Failed to accept client connection: %s",
                             strerror(errno));
                }
            } else {
                // 设置 client socket 非阻塞(仅 TCP 层)
                setNonBlocking(client_fd);

                // 关闭之前的客户端连接(如果存在)
                std::lock_guard<std::mutex> lock(tcp_mutex_);
                if (tcp_client_fd_ >= 0) {
                    close(tcp_client_fd_);
                }
                tcp_client_fd_ = client_fd;

                // 设置接收超时 (5秒,避免长时间阻塞)
                tv.tv_sec = 5;
                tv.tv_usec = 0;
                setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

                // 新客户端连接,清空缓存(仅 TCP 层)
                rx_cache.clear();

                char client_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_addr.sin_addr, client_ip,
                          INET_ADDRSTRLEN);
                ROS_INFO("TCP client connected from %s:%d", client_ip,
                         ntohs(client_addr.sin_port));
            }
        }

        // 处理客户端请求
        client_fd_snapshot = -1;
        {
            std::lock_guard<std::mutex> lock(tcp_mutex_);
            client_fd_snapshot = tcp_client_fd_;
        }

        if (client_fd_snapshot >= 0 &&
            FD_ISSET(client_fd_snapshot, &read_fds)) {
            uint8_t buffer[256];
            ssize_t recv_len =
                recv(client_fd_snapshot, buffer, sizeof(buffer), 0);

            if (recv_len > 0) {
                // TCP 层:追加到缓存,做 Modbus TCP 组帧(解决半包/粘包)
                rx_cache.insert(rx_cache.end(), buffer, buffer + recv_len);

                // Modbus TCP: total_len = 6 + length_field
                while (true) {
                    if (rx_cache.size() < 6) break;

                    uint16_t protocol_id = (rx_cache[2] << 8) | rx_cache[3];
                    uint16_t length_field = (rx_cache[4] << 8) | rx_cache[5];

                    if (protocol_id != 0) {
                        // 非法数据流,丢 1 字节尝试重同步
                        rx_cache.erase(rx_cache.begin());
                        continue;
                    }

                    // length_field 合理范围:至少 unit(1)+func(1)=2,最大 253
                    if (length_field < 2 || length_field > 253) {
                        rx_cache.erase(rx_cache.begin());
                        continue;
                    }

                    size_t total = 6 + static_cast<size_t>(length_field);
                    if (rx_cache.size() < total) break;

                    std::vector<uint8_t> frame(rx_cache.begin(),
                                               rx_cache.begin() + total);
                    rx_cache.erase(rx_cache.begin(), rx_cache.begin() + total);

                    // 处理Modbus请求(此时保证是一整帧)
                    handleModbusRequest(frame.data(),
                                        static_cast<ssize_t>(frame.size()));
                }
            } else if (recv_len == 0) {
                // 错误或客户端断开
                std::lock_guard<std::mutex> lock(tcp_mutex_);
                close(tcp_client_fd_);
                tcp_client_fd_ = -1;
                rx_cache.clear();
                ROS_WARN("TCP client disconnected");
            } else if (recv_len < 0 && errno != EAGAIN &&
                       errno != EWOULDBLOCK) {
                // 错误或客户端断开
                std::lock_guard<std::mutex> lock(tcp_mutex_);
                close(tcp_client_fd_);
                tcp_client_fd_ = -1;
                rx_cache.clear();
                ROS_WARN("TCP client disconnected");
            }
        }
    }

    ROS_INFO("TCP server thread stopped");
}

/**
 * @brief 通过TCP发送目标位置数据,更新Modbus寄存器
 * @param position 目标位置消息
 */
void LidarTargetDetector::sendTCPData(
    const geometry_msgs::PointStamped& position)
{
    std::lock_guard<std::mutex> lock(tcp_mutex_);

    // 更新Modbus寄存器
    // 更新x坐标(米转毫米,int16)
    int16_t x_mm = static_cast<int16_t>(position.point.x * 1000);
    modbus_registers_[REG_X_COORD] = static_cast<uint16_t>(x_mm);

    // 更新y坐标(米转毫米,int16)
    int16_t y_mm = static_cast<int16_t>(position.point.y * 1000);
    modbus_registers_[REG_Y_COORD] = static_cast<uint16_t>(y_mm);

    ROS_DEBUG("Modbus registers updated: heartbeat=%u, x=%dmm, y=%dmm",
              modbus_registers_[REG_HEARTBEAT], x_mm, y_mm);
}

/**
 * @brief 处理Modbus TCP请求
 * @param request Modbus请求数据
 * @param length 请求数据长度
 */
void LidarTargetDetector::handleModbusRequest(uint8_t* request, ssize_t length)
{
    if (length < 8) {
        return; // Modbus TCP最小长度为8字节
    }

    // 解析Modbus TCP头部
    uint16_t transaction_id = (request[0] << 8) | request[1];
    uint16_t protocol_id = (request[2] << 8) | request[3];
    uint16_t length_field = (request[4] << 8) | request[5];
    uint8_t unit_id = request[6];
    uint8_t function_code = request[7];

    // 检查协议ID(Modbus TCP应该是0)
    if (protocol_id != 0) {
        return;
    }

    // 校验长度字段:完整帧应满足 length == 6 + length_field
    if (length != static_cast<ssize_t>(6 + length_field)) {
        return;
    }

    // 准备响应
    uint8_t response[256];
    ssize_t response_len = 0;

    // 复制事务ID、协议ID、单元ID
    response[0] = request[0];
    response[1] = request[1];
    response[2] = request[2];
    response[3] = request[3];
    response[6] = unit_id;

    switch (function_code) {
        case 0x03: // 读保持寄存器
        {
            if (length < 12) {
                return;
            }

            uint16_t start_addr = (request[8] << 8) | request[9];
            uint16_t reg_count = (request[10] << 8) | request[11];

            // 合法范围:1~125
            if (reg_count == 0 || reg_count > 125) {
                // 返回异常响应
                response[7] = 0x83; // 异常功能码
                response[8] = 0x03; // 非法数据值
                response_len = 9;
                break;
            }

            // 检查寄存器范围
            if (start_addr + reg_count >
                sizeof(modbus_registers_) / sizeof(uint16_t)) {
                // 返回异常响应
                response[7] = 0x83; // 异常功能码
                response[8] = 0x02; // 非法数据地址
                response_len = 9;
            } else {
                response[7] = 0x03;
                response[8] = reg_count * 2; // 字节数
                response_len = 9;

                // 复制寄存器数据
                {
                    std::lock_guard<std::mutex> lock(tcp_mutex_);
                    for (uint16_t i = 0; i < reg_count; i++) {
                        uint16_t reg_value = modbus_registers_[start_addr + i];
                        response[response_len++] = (reg_value >> 8) & 0xFF;
                        response[response_len++] = reg_value & 0xFF;
                    }
                }
            }
            break;
        }
        case 0x06: // 写单个寄存器
        {
            if (length < 12) {
                return;
            }

            uint16_t reg_addr = (request[8] << 8) | request[9];
            uint16_t reg_value = (request[10] << 8) | request[11];

            // 检查寄存器范围
            if (reg_addr >= sizeof(modbus_registers_) / sizeof(uint16_t)) {
                response[7] = 0x86; // 异常功能码
                response[8] = 0x02; // 非法数据地址
                response_len = 9;
            } else {
                // 写入寄存器
                {
                    std::lock_guard<std::mutex> lock(tcp_mutex_);
                    modbus_registers_[reg_addr] = reg_value;
                }

                // 返回写入确认
                response[7] = 0x06;
                response[8] = (reg_addr >> 8) & 0xFF;
                response[9] = reg_addr & 0xFF;
                response[10] = (reg_value >> 8) & 0xFF;
                response[11] = reg_value & 0xFF;
                response_len = 12;
            }
            break;
        }
        default:
            // 不支持的函数码
            response[7] = function_code | 0x80;
            response[8] = 0x01; // 非法函数
            response_len = 9;
            break;
    }

    // 统一更新长度字段(包括异常响应)
    response[4] = ((response_len - 6) >> 8) & 0xFF;
    response[5] = (response_len - 6) & 0xFF;

    // 发送响应
    int fd_snapshot = -1;
    {
        std::lock_guard<std::mutex> lock(tcp_mutex_);
        fd_snapshot = tcp_client_fd_;
    }

    if (fd_snapshot >= 0) {
        bool ok =
            sendAll(fd_snapshot, response, static_cast<size_t>(response_len));
        if (!ok) {
            ROS_WARN("Failed to send Modbus response: %s", strerror(errno));
            std::lock_guard<std::mutex> lock(tcp_mutex_);
            close(tcp_client_fd_);
            tcp_client_fd_ = -1;
        }
    }
}