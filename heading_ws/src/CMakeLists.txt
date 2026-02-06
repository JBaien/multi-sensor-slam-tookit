cmake_minimum_required(VERSION 3.10)
project(heading_estimation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_ros
  tf
)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    std_msgs
    pcl_ros
    tf
  LIBRARIES heading_estimation_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# Library
add_library(heading_estimation_lib
  src/HeadingEstimator.cpp
  src/PointCloudProcessing.cpp
  src/Config.cpp
)

target_link_libraries(heading_estimation_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Node
add_executable(heading_estimation_node
  src/heading_estimation_node.cpp
)

target_link_libraries(heading_estimation_node
  heading_estimation_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Optional: Compile test (uncomment to enable)
# add_executable(compile_test test/compile_test.cpp)
# target_link_libraries(compile_test heading_estimation_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Install
install(TARGETS heading_estimation_lib heading_estimation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
