#include <geometry_msgs/Vector3Stamped.h>
#include <locale.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>

#include <clocale>
#include <memory>
#include <mutex>
#include <thread>

#include "heading_estimation/HeadingEstimator.h"

namespace heading_estimation {

/**
 * @brief 单激光雷达航向和墙距估计的ROS节点
 */
class HeadingEstimationNode {
   public:
    HeadingEstimationNode()
        : nh_("~"), tf_listener_(), processing_thread_running_(false)
    {
        // 加载配置
        config_.loadFromROS(nh_);

        // 创建估计器
        estimator_ = std::make_unique<HeadingEstimator>(config_);

        // 初始化 TF 变换
        initTFTransform();

        // 创建发布器
        attitude_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
            config_.attitude_topic, 10);
        distances_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
            config_.distances_topic, 10);

        // 创建订阅器
        pointcloud_sub_ =
            nh_.subscribe(config_.pointcloud_topic, 1,
                          &HeadingEstimationNode::pointCloudCallback, this);

        // 统计计数器
        frame_count_ = 0;
        success_count_ = 0;
        last_print_time_ = ros::Time::now();

        ROS_INFO("航向估计节点已初始化");
        ROS_INFO("输入话题: %s", config_.pointcloud_topic.c_str());
        ROS_INFO("输出话题: %s, %s", config_.attitude_topic.c_str(),
                 config_.distances_topic.c_str());
    }

    ~HeadingEstimationNode()
    {
        processing_thread_running_ = false;
        if (processing_thread_ && processing_thread_->joinable()) {
            processing_thread_->join();
        }
    }

   private:
    void initTFTransform()
    {
        // 检查静态变换是否已存在
        bool transform_exists = true;
        try {
            tf_listener_.waitForTransform(config_.base_frame,
                                          config_.lidar_frame, ros::Time(0),
                                          ros::Duration(1.0));
        } catch (tf::TransformException& ex) {
            transform_exists = false;
        }

        if (!transform_exists) {
            ROS_WARN("未找到 %s 和 %s 之间的静态 TF 变换",
                     config_.lidar_frame.c_str(), config_.base_frame.c_str());
            ROS_WARN("假设 lidar_frame == base_frame");
            ROS_WARN("如果坐标系不同，请发布静态变换");
        }
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        try {
            // 转换为 PCL 点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);

            // 检查帧是否匹配
            if (msg->header.frame_id != config_.base_frame &&
                msg->header.frame_id != config_.lidar_frame) {
                ROS_WARN_THROTTLE(
                    5.0, "点云坐标系 %s 与期望的坐标系 [%s, %s] 不匹配",
                    msg->header.frame_id.c_str(), config_.base_frame.c_str(),
                    config_.lidar_frame.c_str());
            }

            // 如需要则变换到 base_frame
            if (msg->header.frame_id == config_.lidar_frame &&
                config_.base_frame != config_.lidar_frame) {
                tf::StampedTransform transform;
                try {
                    tf_listener_.lookupTransform(config_.base_frame,
                                                 config_.lidar_frame,
                                                 msg->header.stamp, transform);

                    pcl_ros::transformPointCloud(config_.base_frame, *cloud,
                                                 *cloud, tf_listener_);
                } catch (tf::TransformException& ex) {
                    ROS_ERROR_THROTTLE(2.0, "TF 查找失败: %s", ex.what());
                    return;
                }
            }

            // 处理点云
            ros::Time start_time = ros::Time::now();
            EstimationResult result =
                estimator_->processPointCloud(cloud, msg->header.stamp);
            ros::Time end_time = ros::Time::now();

            // 发布结果
            publishResults(result, msg->header.stamp);

            // 更新统计
            ++frame_count_;
            if (result.confidence_high) {
                ++success_count_;
            }

            // 定期打印统计
            if ((end_time - last_print_time_).toSec() >= 5.0) {
                printStatistics(end_time - start_time);
                last_print_time_ = end_time;
            }

        } catch (const std::exception& ex) {
            ROS_ERROR("点云回调异常: %s", ex.what());
        }
    }

    void publishResults(const EstimationResult& result,
                        const ros::Time& timestamp)
    {
        // 发布姿态
        geometry_msgs::Vector3Stamped attitude_msg;
        attitude_msg.header.stamp = timestamp;
        attitude_msg.header.frame_id = config_.base_frame;
        attitude_msg.vector.x = result.roll;
        attitude_msg.vector.y = result.pitch;
        attitude_msg.vector.z = result.yaw;
        attitude_pub_.publish(attitude_msg);

        // 发布距离
        std_msgs::Float32MultiArray distances_msg;
        distances_msg.data.resize(result.distances.size());
        for (size_t i = 0; i < result.distances.size(); ++i) {
            distances_msg.data[i] = static_cast<float>(result.distances[i]);
        }
        distances_pub_.publish(distances_msg);

        // 低频日志
        if (frame_count_ % 20 == 0) {
            ROS_INFO("姿态角: Roll=%.3f°, Pitch=%.3f°, Yaw=%.3f°", result.roll,
                     result.pitch, result.yaw);
            ROS_INFO("墙距 [m]: 左前=%.2f, 左后=%.2f, 右前=%.2f, 右后=%.2f",
                     result.distances[0], result.distances[1],
                     result.distances[2], result.distances[3]);

            std::string confidence_level = result.confidence_high     ? "高"
                                           : result.confidence_medium ? "中"
                                                                      : "低";
            ROS_INFO("置信度: %s (地面内点数=%d, RMS=%.3fm)",
                     confidence_level.c_str(), result.ground_inliers,
                     result.ground_rms);
        }
    }

    void printStatistics(const ros::Duration& processing_time)
    {
        double success_rate =
            (frame_count_ > 0) ? 100.0 * success_count_ / frame_count_ : 0.0;

        ROS_INFO("=== 统计信息 ===");
        ROS_INFO("已处理帧数: %d", frame_count_);
        ROS_INFO("高置信度率: %.1f%%", success_rate);
        ROS_INFO("处理时间: %.3f ms (%.1f Hz)",
                 processing_time.toSec() * 1000.0,
                 1.0 / processing_time.toSec());
        ROS_INFO("=================");
    }

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;

    HeadingConfig config_;
    std::unique_ptr<HeadingEstimator> estimator_;

    ros::Subscriber pointcloud_sub_;
    ros::Publisher attitude_pub_;
    ros::Publisher distances_pub_;

    // Thread for async processing
    std::unique_ptr<std::thread> processing_thread_;
    bool processing_thread_running_;
    std::mutex queue_mutex_;

    // Statistics
    int frame_count_;
    int success_count_;
    ros::Time last_print_time_;
};

} // namespace heading_estimation

int main(int argc, char** argv)
{
    // 强制设置 UTF-8 编码以支持中文显示
    setlocale(LC_ALL, "zh_CN.UTF-8");

    ros::init(argc, argv, "heading_estimation_node");

    try {
        heading_estimation::HeadingEstimationNode node;
        ROS_INFO("航向估计节点已就绪");
        ros::spin();

    } catch (const std::exception& ex) {
        ROS_FATAL("致命异常: %s", ex.what());
        return 1;
    }

    return 0;
}
