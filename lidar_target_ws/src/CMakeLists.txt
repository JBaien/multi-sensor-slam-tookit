cmake_minimum_required(VERSION 3.0.2)
project(lidar_target_detection)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  tf
  eigen_conversions
)

# catkin 包配置 - 必须在 add_executable 之前
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    pcl_ros 
    pcl_conversions 
    tf 
    eigen_conversions
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(lidar_target_detector
  src/lidar_target_detector.cpp
)

# 链接库
target_link_libraries(lidar_target_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 安装可执行文件
install(TARGETS lidar_target_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 安装 launch 文件夹
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
