#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

struct PointXYZIRT
{
  float x;
  float y;
  float z;
  float intensity;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring)
  (float, time, time)
)

template<typename PointT>
void removeNaNFromOrganizedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                      pcl::PointCloud<PointT> &cloud_out)
{
  cloud_out = cloud_in;

  for (auto& point : cloud_out.points)
  {
    if (!pcl::isFinite(point))
    {
      point.x = 0.0f;
      point.y = 0.0f;
      point.z = 0.0f;
      point.intensity = 0.0f;
      // ring 和 time 保留
    }
  }

  cloud_out.width = cloud_in.width;
  cloud_out.height = cloud_in.height;
  cloud_out.is_dense = true;  // 关键
}

class OrganizedNanFilterNode
{
public:
  OrganizedNanFilterNode()
  {
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("input_topic", input_topic_, "/lidar_points");
    private_nh.param<std::string>("output_topic", output_topic_, "/velodyne_points_old");

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    sub_ = nh_.subscribe(input_topic_, 1, &OrganizedNanFilterNode::cloudCallback, this);

    ROS_INFO("Organized NaN Filter Node started:");
    ROS_INFO_STREAM("  Subscribing: " << input_topic_);
    ROS_INFO_STREAM("  Publishing : " << output_topic_);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::PointCloud<PointXYZIRT>::Ptr cloud(new pcl::PointCloud<PointXYZIRT>());
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIRT>());

    pcl::fromROSMsg(*cloud_msg, *cloud);
    removeNaNFromOrganizedPointCloud(*cloud, *cloud_filtered);
    
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_msg);
    filtered_msg.header.frame_id = "velodyne";
    if (cloud_msg->header.stamp.isZero()) {
        filtered_msg.header.stamp = ros::Time::now();
        //ROS_INFO("Publishing filtered message with timestamp: %f", filtered_msg.header.stamp.toSec());
    } else {
        filtered_msg.header.stamp = cloud_msg->header.stamp;
    }
    pub_.publish(filtered_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::string input_topic_;
  std::string output_topic_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "organized_nan_filter_node");
  OrganizedNanFilterNode node;
  ros::spin();
  return 0;
} 
