# LiDAR Fusion

一个用于融合多个激光雷达点云数据的ROS功能包。

## 功能特性

- 支持融合两个激光雷达传感器的点云数据
- 自动应用TF变换将不同坐标系的点云转换到统一坐标系
- 使用近似时间同步确保数据一致性
- 支持自定义输入/输出topic和坐标系frame_id
- 完整的错误处理和日志记录

## 系统要求

- ROS Kinetic/Melodic/Noetic
- PCL (Point Cloud Library) 1.7+
- Eigen3
- C++11编译器

## 依赖包

```bash
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions ros-$ROS_DISTRO-tf2-eigen
```

## 编译安装

1. 将此包放入catkin工作空间的src目录
2. 编译包：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 基本使用

使用默认参数启动融合节点：

```bash
roslaunch lidar_fusion lidar_fusion.launch
```

### 自定义参数

您可以在launch文件中修改以下参数：

- `lslidar_topic`: LSLidar点云数据topic (默认: `/cx/lslidar_point_cloud`)
- `velodyne_topic`: Velodyne点云数据topic (默认: `/velodyne_points`)
- `output_topic`: 融合后的点云数据topic (默认: `/fused_point_cloud`)
- `output_frame_id`: 输出点云的坐标系frame_id (默认: `velodyne`)
- `tf_tolerance`: TF变换容忍度 (默认: `0.1`秒)

### 自定义topic和frame_id

修改launch文件中的参数：

```xml
<param name="lslidar_topic" value="/your/lslidar/topic" />
<param name="velodyne_topic" value="/your/velodyne/topic" />
<param name="output_topic" value="/your/output/topic" />
<param name="output_frame_id" value="your_frame_id" />
```

### TF变换配置

本包包含预配置的静态TF变换发布器，参数为：
- x: 0.00903051
- y: 0.243625  
- z: 0.0100529
- yaw: 0.0586917
- pitch: -0.00286561
- roll: -0.000914872

从 `/velodyne` 坐标系到 `/laser_link` 坐标系的变换。

如需修改TF变换参数，请编辑launch文件中的static_transform_publisher节点。

## 话题说明

### 订阅的话题

- `{lslidar_topic}` (sensor_msgs/PointCloud2): LSLidar点云数据
- `{velodyne_topic}` (sensor_msgs/PointCloud2): Velodyne点云数据

### 发布的话题

- `{output_topic}` (sensor_msgs/PointCloud2): 融合后的点云数据

## 参数说明

- `~lslidar_topic` (string): LSLidar点云输入topic名称
- `~velodyne_topic` (string): Velodyne点云输入topic名称  
- `~output_topic` (string): 融合点云输出topic名称
- `~output_frame_id` (string): 输出点云坐标系frame_id
- `~tf_tolerance` (double): TF变换查找的时间容忍度

## 点云数据格式

支持的点云数据结构包含以下字段：
- x, y, z: 3D坐标
- intensity: 强度值
- ring: 激光环号
- time: 时间戳

## 可视化

使用RVIZ查看融合结果：

```bash
rosrun rviz rviz
```

在RVIZ中添加PointCloud2显示，设置topic为融合后的点云topic。

## 故障排除

### 常见问题

1. **TF变换错误**
   - 确保static_transform_publisher正在运行
   - 检查坐标系frame_id是否正确
   - 验证TF树结构：`rosrun tf view_frames`

2. **数据同步问题**
   - 检查输入topic是否有数据：`rostopic hz /topic_name`
   - 调整tf_tolerance参数
   - 确保两个传感器的时间戳基本同步

3. **编译错误**
   - 确保所有依赖包已安装
   - 检查PCL版本兼容性
   - 验证Eigen3安装

### 调试信息

启用调试输出：

```bash
roslaunch lidar_fusion lidar_fusion.launch --screen
```

或设置日志级别：

```bash
rosparam set /lidar_fusion_node/rosconsole/logger_level DEBUG
```

## 许可证

本项目采用BSD许可证。

## 贡献

欢迎提交问题报告和功能请求。

## 作者

用户开发维护。