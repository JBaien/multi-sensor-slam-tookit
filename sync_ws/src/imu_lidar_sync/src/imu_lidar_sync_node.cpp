#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <deque>
#include <mutex>

class ImuLidarSync
{
public:
    ImuLidarSync(ros::NodeHandle& nh)
    {
        imu_sub_ = nh.subscribe("/imu/olddata", 500, &ImuLidarSync::imuCallback, this);
        lidar_sub_ = nh.subscribe("/old_velodyne_points", 50, &ImuLidarSync::lidarCallback, this);

        imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 500);
        lidar_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
    }

private:
    void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_buffer_.push_back(*imu_msg);
        if (imu_buffer_.size() > 2000)  // 增大缓存，存足够多帧
            imu_buffer_.pop_front();

        // 高频发布
        imu_pub_.publish(*imu_msg);
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (imu_buffer_.empty())
        {
            ROS_WARN("No IMU data in buffer!");
            return;
        }

        // 收集所有时间 <= LiDAR 时间的 IMU 帧
        std::deque<sensor_msgs::Imu> imu_for_lidar;
        auto it = imu_buffer_.begin();
        while (it != imu_buffer_.end())
        {
            if (it->header.stamp <= lidar_msg->header.stamp)
            {
                imu_for_lidar.push_back(*it);
                it = imu_buffer_.erase(it);  // 用过的 IMU 可以删除
            }
            else
            {
                ++it;
            }
        }

        if (imu_for_lidar.empty())
        {
            ROS_WARN("No IMU frames matched LiDAR timestamp!");
            return;
        }

        // 发布 LiDAR
        lidar_pub_.publish(*lidar_msg);

        // 日志显示同步信息
        ROS_INFO("LiDAR %.6f synced with %lu IMU frames. first: %.6f, last: %.6f",
                 lidar_msg->header.stamp.toSec(),
                 imu_for_lidar.size(),
                 imu_for_lidar.front().header.stamp.toSec(),
                 imu_for_lidar.back().header.stamp.toSec());

        // 这里你可以把 imu_for_lidar 传给 LIO-SAM 或其他处理
    }

    ros::Subscriber imu_sub_;
    ros::Subscriber lidar_sub_;
    ros::Publisher imu_pub_;
    ros::Publisher lidar_pub_;

    std::deque<sensor_msgs::Imu> imu_buffer_;
    std::mutex mutex_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_lidar_sync_node");
    ros::NodeHandle nh;

    ImuLidarSync sync_node(nh);

    ros::spin();
    return 0;
}

