cmake_minimum_required(VERSION 3.0.2)
project(imu_modbus_driver)

add_compile_options(-std=c++11)

## 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs    # 如果用到 Quaternion/Vector3
  nav_msgs         # 如果发布 Odometry
  tf
  tf2
  tf2_ros
)

# 查找 libmodbus
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# 查找 Eigen
find_package(Eigen3 REQUIRED)

## 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs tf tf2 tf2_ros
)

## 包含头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBMODBUS_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 添加可执行程序
add_executable(imu_modbus_node src/imu_modbus_node.cpp)

## 链接依赖库
target_link_libraries(imu_modbus_node
  ${catkin_LIBRARIES}
  ${LIBMODBUS_LIBRARIES}
)

