# 单激光雷达掘进机姿态与墙距实时估计系统 (无IMU)

基于ROS1和C++实现的单台三维激光雷达SLAM算法模块，用于井下巷道环境中实时估计掘进机的航向角、俯仰角、横滚角，并输出中轴线上前后2m位置到左右墙面的四个垂直距离。

## 特性

- **无IMU依赖**：仅使用激光雷达点云数据，不依赖IMU、轮速或UWB等外部传感器
- **姿态估计**：实时输出roll、pitch、yaw三轴姿态角（ZYX欧拉角约定）
- **墙距测量**：输出左前、左后、右前、右后四个距离值
- **鲁棒性强**：支持墙面材质变化、地面不平整、点密度变化等复杂场景
- **工程化设计**：完善的日志系统、异常处理、性能优化和质量控制

## 系统要求

- ROS 1 (Melodic/Noetic)
- C++14或更高版本
- PCL 1.7+
- Eigen3

## 安装

### 1. 克隆到catkin工作空间

```bash
cd ~/catkin_ws/src
git clone <repository_url> heading_estimation
cd ~/catkin_ws
```

### 2. 安装依赖

```bash
sudo apt-get update
sudo apt-get install ros-$(rosversion -d)-pcl-ros
sudo apt-get install ros-$(rosversion -d)-tf
sudo apt-get install libeigen3-dev
```

### 3. 编译

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 配置

### 坐标系与轴向配置

系统使用以下坐标系约定：

1. **雷达坐标系**：Z轴向上
2. **base_frame**：掘进机本体坐标系
3. **forward_axis**：前进方向轴，取值为 `+X`、`-X`、`+Y`、`-Y` 之一
4. **left_axis**：由右手系定义为 `left_axis = Z × forward_axis`

示例：
- `forward_axis = +X` 时，`left_axis = +Y`
- `forward_axis = +Y` 时，`left_axis = -X`

### 参数配置

#### 主要配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pointcloud_topic` | `/velodyne_points` | 输入点云话题 |
| `base_frame` | `base_link` | 掘进机本体坐标系 |
| `lidar_frame` | `velodyne` | 雷达坐标系 |
| `forward_axis` | `+X` | 前进方向轴 |
| `sample_front_dist` | `2.0` | 中轴线前方采样距离(m) |
| `sample_back_dist` | `-2.0` | 中轴线后方采样距离(m) |
| `ground_roi_radius` | `3.0` | 地面拟合ROI半径(m) |
| `wall_z_min` | `0.5` | 墙面提取最小高度(m) |
| `wall_z_max` | `3.0` | 墙面提取最大高度(m) |
| `voxel_size` | `0.05` | 体素滤波尺寸(m) |
| `temporal_smoothing_tau` | `0.3` | 时间平滑常数(s) |

#### 完整配置文件

参考 `config/default_params.yaml` 获取完整的参数列表和说明。

## 使用方法

### 启动节点

```bash
# 使用默认参数
roslaunch heading_estimation heading_estimation.launch

# 指定话题和坐标系
roslaunch heading_estimation heading_estimation.launch \
    pointcloud_topic:=/your_lidar_topic \
    base_frame:=your_base_frame \
    lidar_frame:=your_lidar_frame \
    forward_axis:=+Y
```

### 发布静态TF变换（如需要）

如果雷达坐标系与base_frame不同，需要发布静态变换：

```bash
# 格式: x y z yaw pitch roll lidar_frame base_frame period_in_ms
rosrun tf static_transform_publisher \
    0 0 0.5 0 0 0 \
    velodyne base_link \
    100
```

或在launch文件中取消注释相关节点。

## 输出话题

### /pose_wall/attitude (geometry_msgs/Vector3Stamped)

输出姿态角，单位：角度

- `x`: Roll（绕X轴旋转，度）
- `y`: Pitch（绕Y轴旋转，度）
- `z`: Yaw（绕Z轴旋转，航向角，度）

### /pose_wall/distances (std_msgs/Float32MultiArray)

输出四个墙距，单位：米，顺序为：

- `data[0]`: 左前距离 (LF, Left-Front)
- `data[1]`: 左后距离 (LB, Left-Back)
- `data[2]`: 右前距离 (RF, Right-Front)
- `data[3]`: 右后距离 (RB, Right-Back)

距离值 `-1.0` 表示该侧墙面检测失败。

## 算法流程

```
1. 点云接收与过滤
   ├─ 体素滤波降采样
   └─ 统计离群点移除

2. 地面平面估计（局部ROI）
   ├─ 高度分位数筛选低点
   ├─ RANSAC平面拟合
   ├─ Tukey加权最小二乘精修
   └─ 提取Roll和Pitch

3. 点云水平化变换
   └─ 使用R_level变换点云到水平坐标系

4. 墙面平面提取
   ├─ 高度过滤 [wall_z_min, wall_z_max]
   ├─ RANSAC垂直平面提取
   ├─ 左右墙面判定
   └─ 航向角(Yaw)计算

5. 墙距计算
   ├─ 中轴线采样点定义
   ├─ 点到平面距离计算
   └─ 输出四个距离

6. 质量控制与平滑
   ├─ 拟合残差检查
   ├─ 时域低通滤波
   └─ 退化检测与处理
```

## 可视化

### 使用RViz

```bash
rviz
```

添加以下内容进行可视化：

1. **PointCloud2**: 添加输入点云
2. **Vector3Stamped**: 添加姿态话题
3. **Array**: 添加墙距话题（可使用rqt_plot）

### 使用rqt_plot查看数据

```bash
rqt_plot /pose_wall/attitude/vector /pose_wall/distances/data
```

## 性能

在典型ARM64工控平台上的性能表现：

- **输入点云**：约10-20万点/帧
- **处理后点数**：约2-5万点
- **处理频率**：10-20 Hz
- **处理延迟**：约50-100ms

## 质量控制

系统包含多层质量控制：

1. **地面平面**：检查RMS误差和内点数量
2. **墙面平面**：检查垂直性、平行性和拟合质量
3. **航向角**：限制单帧变化率
4. **置信度等级**：输出HIGH/MEDIUM/LOW三级置信度标志

## 故障排除

### 问题：地面平面估计失败

**可能原因**：
- 地面ROI半径设置过小
- 地面过于不平整
- 点云质量差

**解决方案**：
```bash
# 增大ground_roi_radius
rosparam set /heading_estimation_node/ground_roi_radius 5.0
```

### 问题：墙面检测失败

**可能原因**：
- 墙面高度范围设置不当
- 墙面点云密度不足
- 雷达安装位置不佳

**解决方案**：
```bash
# 调整墙面高度范围
rosparam set /heading_estimation_node/wall_z_min 0.3
rosparam set /heading_estimation_node/wall_z_max 3.5

# 降低墙面最小内点数
rosparam set /heading_estimation_node/wall_inlier_min 200
```

### 问题：姿态输出抖动

**可能原因**：
- 时间平滑常数过小
- 点云质量不稳定

**解决方案**：
```bash
# 增大平滑常数
rosparam set /heading_estimation_node/temporal_smoothing_tau 0.5

# 限制最大航向角变化率
rosparam set /heading_estimation_node/max_yaw_rate 0.03  # ~1.7 deg/frame
```

### 问题：处理速度慢

**解决方案**：
```bash
# 增大体素滤波尺寸（降低精度，提高速度）
rosparam set /heading_estimation_node/voxel_size 0.08

# 减小ROI半径
rosparam set /heading_estimation_node/ground_roi_radius 2.5
```

## 日志级别

调整日志输出频率：

```bash
# INFO（默认）
rosservice call /heading_estimation_node/set_logger_level ros.heading_estimation_node info

# DEBUG（详细调试信息）
rosservice call /heading_estimation_node/set_logger_level ros.heading_estimation_node debug

# WARN（仅警告和错误）
rosservice call /heading_estimation_node/set_logger_level ros.heading_estimation_node warn
```

## 扩展开发

### 添加自定义参数

1. 在 `HeadingConfig` 结构中添加参数
2. 在 `loadFromROS` 方法中加载参数
3. 在launch文件或yaml配置文件中设置

### 集成到现有系统

1. 确保TF树中存在 `base_frame` 和 `lidar_frame`
2. 修改 `forward_axis` 匹配实际车辆朝向
3. 根据实际尺寸调整 `sample_front_dist` 和 `sample_back_dist`

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request。

## 联系方式

如有问题或建议，请联系开发团队。

## 更新日志

### v1.0.0 (2026-02-04)
- 初始版本发布
- 实现完整的单激光雷达姿态与墙距估计算法
- 支持工程化落地

rosrun tf static_transform_publisher 0 0 0 0 0 0 timoo world 100
