cmake_minimum_required(VERSION 3.0.2)
project(lidar_reflector_target_tracker)

add_compile_options(-O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

find_package(PCL REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(lidar_reflector_target_tracker_node
  src/main.cpp
  src/node.cpp
  src/circle_fit.cpp
  src/kalman_filter.cpp
)

target_link_libraries(lidar_reflector_target_tracker_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

## Install ##
install(TARGETS lidar_reflector_target_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
