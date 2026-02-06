cmake_minimum_required(VERSION 3.0.2)
project(imu_lidar_sync)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_lidar_sync_node src/imu_lidar_sync_node.cpp)

target_link_libraries(imu_lidar_sync_node
  ${catkin_LIBRARIES}
)

install(TARGETS imu_lidar_sync_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

