#include "heading_estimation/Config.h"

#include <ros/ros.h>

#include <cmath>

namespace heading_estimation {

HeadingConfig::HeadingConfig()
    : pointcloud_topic("/velodyne_points"),
      base_frame("base_link"),
      lidar_frame("velodyne"),
      attitude_topic("/pose_wall/attitude"),
      distances_topic("/pose_wall/distances"),
      forward_axis("+X"),
      forward_vector(1.0, 0.0, 0.0),
      vehicle_length(4.5),
      vehicle_width(1.2),
      track_length(1.4),
      track_width(1.0),
      sample_front_dist(2.0),
      sample_back_dist(-2.0),
      ground_roi_radius(3.0),
      ground_z_percentile(0.2),
      ground_inlier_min(500),
      ground_normal_threshold(5.0),
      ground_ransac_max_iter(200),
      ground_ransac_threshold(0.05),
      plane_inlier_threshold(0.05),
      wall_z_min(0.5),
      wall_z_max(3.0),
      wall_z_percentile_min(0.2),
      wall_z_percentile_max(0.8),
      wall_use_percentile_filter(true),
      wall_inlier_min(300),
      wall_vertical_threshold(0.2),
      wall_parallel_cos(0.9),
      wall_max_planes(4),
      wall_side_cos_threshold(0.3),
      wall_ransac_max_iter(300),
      wall_ransac_threshold(0.05),
      wall_distance_max(100.0),
      voxel_size(0.05),
      outlier_mean_k(20),
      outlier_std_mul(1.5),
      temporal_smoothing_tau(0.3),
      max_yaw_rate(3.0 * M_PI / 180.0),
      ground_max_rms(0.1),
      wall_max_rms(0.15)
{
}

void HeadingConfig::loadFromROS(ros::NodeHandle& nh)
{
    nh.param<std::string>("pointcloud_topic", pointcloud_topic,
                          pointcloud_topic);
    nh.param<std::string>("base_frame", base_frame, base_frame);
    nh.param<std::string>("lidar_frame", lidar_frame, lidar_frame);
    nh.param<std::string>("attitude_topic", attitude_topic, attitude_topic);
    nh.param<std::string>("distances_topic", distances_topic, distances_topic);
    nh.param<std::string>("forward_axis", forward_axis, forward_axis);
    // 解析前进轴
    if (forward_axis == "+X" || forward_axis == "+x") {
        forward_vector = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else if (forward_axis == "-X" || forward_axis == "-x") {
        forward_vector = Eigen::Vector3d(-1.0, 0.0, 0.0);
    } else if (forward_axis == "+Y" || forward_axis == "+y") {
        forward_vector = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else if (forward_axis == "-Y" || forward_axis == "-y") {
        forward_vector = Eigen::Vector3d(0.0, -1.0, 0.0);
    } else {
        ROS_WARN("Invalid forward_axis: %s, using +X", forward_axis.c_str());
        forward_vector = Eigen::Vector3d(1.0, 0.0, 0.0);
    }

    nh.param<double>("vehicle_length", vehicle_length, vehicle_length);
    nh.param<double>("vehicle_width", vehicle_width, vehicle_width);
    nh.param<double>("track_length", track_length, track_length);
    nh.param<double>("track_width", track_width, track_width);

    nh.param<double>("sample_front_dist", sample_front_dist, sample_front_dist);
    nh.param<double>("sample_back_dist", sample_back_dist, sample_back_dist);
    nh.param<double>("ground_roi_radius", ground_roi_radius, ground_roi_radius);
    nh.param<double>("ground_z_percentile", ground_z_percentile,
                     ground_z_percentile);
    nh.param<int>("ground_inlier_min", ground_inlier_min, ground_inlier_min);
    nh.param<double>("ground_normal_threshold", ground_normal_threshold,
                     ground_normal_threshold);
    nh.param<int>("ground_ransac_max_iter", ground_ransac_max_iter,
                  ground_ransac_max_iter);
    nh.param<double>("ground_ransac_threshold", ground_ransac_threshold,
                     ground_ransac_threshold);
    nh.param<double>("plane_inlier_threshold", plane_inlier_threshold,
                     plane_inlier_threshold);
    nh.param<double>("wall_z_min", wall_z_min, wall_z_min);
    nh.param<double>("wall_z_max", wall_z_max, wall_z_max);
    nh.param<double>("wall_z_percentile_min", wall_z_percentile_min,
                     wall_z_percentile_min);
    nh.param<double>("wall_z_percentile_max", wall_z_percentile_max,
                     wall_z_percentile_max);
    nh.param<bool>("wall_use_percentile_filter", wall_use_percentile_filter,
                   wall_use_percentile_filter);
    nh.param<int>("wall_inlier_min", wall_inlier_min, wall_inlier_min);
    nh.param<double>("wall_vertical_threshold", wall_vertical_threshold,
                     wall_vertical_threshold);
    nh.param<double>("wall_parallel_cos", wall_parallel_cos, wall_parallel_cos);
    nh.param<int>("wall_max_planes", wall_max_planes, wall_max_planes);
    nh.param<double>("wall_side_cos_threshold", wall_side_cos_threshold,
                     wall_side_cos_threshold);
    nh.param<int>("wall_ransac_max_iter", wall_ransac_max_iter,
                  wall_ransac_max_iter);
    nh.param<double>("wall_ransac_threshold", wall_ransac_threshold,
                     wall_ransac_threshold);
    nh.param<double>("wall_distance_max", wall_distance_max, wall_distance_max);
    nh.param<double>("voxel_size", voxel_size, voxel_size);
    nh.param<int>("outlier_mean_k", outlier_mean_k, outlier_mean_k);
    nh.param<double>("outlier_std_mul", outlier_std_mul, outlier_std_mul);
    nh.param<double>("temporal_smoothing_tau", temporal_smoothing_tau,
                     temporal_smoothing_tau);
    nh.param<double>("max_yaw_rate", max_yaw_rate, max_yaw_rate);
    nh.param<double>("ground_max_rms", ground_max_rms, ground_max_rms);
    nh.param<double>("wall_max_rms", wall_max_rms, wall_max_rms);

    // 打印已加载的配置
    ROS_INFO("=== Heading Estimator Configuration ===");
    ROS_INFO("Forward axis: %s, vector: [%.2f, %.2f, %.2f]",
             forward_axis.c_str(), forward_vector.x(), forward_vector.y(),
             forward_vector.z());
    ROS_INFO("Sample points: front=%.2fm, back=%.2fm", sample_front_dist,
             sample_back_dist);
    ROS_INFO("Ground ROI radius: %.2fm, inlier min: %d", ground_roi_radius,
             ground_inlier_min);
    ROS_INFO("Wall height range: [%.2f, %.2f]m, inlier min: %d", wall_z_min,
             wall_z_max, wall_inlier_min);
    ROS_INFO("Voxel size: %.3fm, outlier: k=%d, std=%.2f", voxel_size,
             outlier_mean_k, outlier_std_mul);
    ROS_INFO("=======================================");
}

} // namespace heading_estimation
