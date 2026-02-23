cmake_minimum_required(VERSION 3.0.2)
project(nan_filter)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(organized_nan_filter_node src/nan_filter_node.cpp)

target_link_libraries(organized_nan_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(organized_nan_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

