#include <ros/ros.h>
#include "tracker/node.h"

/**
 * @brief 主函数
 *
 * 初始化ROS节点，创建跟踪器节点实例并运行
 *
 * @param argc 参数个数
 * @param argv 参数列表
 * @return 程序退出码
 */
int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "lidar_reflector_target_tracker");

  // 创建跟踪器节点
  tracker::TargetTrackerNode node;

  // 初始化节点
  node.init();

  // 运行节点(进入ROS事件循环)
  node.run();

  return 0;
}
