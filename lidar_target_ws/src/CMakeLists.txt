cmake_minimum_required(VERSION 3.0.2)
project(lidar_target_detection)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# (可选) 编译警告/优化
add_compile_options(-Wall -Wextra)

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

# 显式找 PCL / Eigen（关键）
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

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
  DEPENDS PCL Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(lidar_target_detector
  src/lidar_target_detector.cpp
)

target_link_libraries(lidar_target_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 有些 PCL 版本需要这个
add_definitions(${PCL_DEFINITIONS})

install(TARGETS lidar_target_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
