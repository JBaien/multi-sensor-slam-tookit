# Docker 部署工具包

本目录包含用于构建和部署 ROS 应用的 Docker 相关文件。

## 目录结构

```
Docker_Tookit/
├── Dockerfile.cross-arch    # 跨架构 Dockerfile (x86_64 -> ARM64)
├── docker-compose.yml        # Docker Compose 配置文件
├── build-cross-arch.sh       # 跨架构镜像构建脚本
├── rebuild-code.sh           # 快速重建脚本（仅重新编译代码）
├── run-docker.sh            # Docker 容器启动脚本
├── check-deployment.sh      # 部署环境检查脚本
├── buildkitd.toml           # Docker Buildx 配置文件
└── .dockerignore            # Docker 构建忽略文件
```

## 使用说明

### 1. 环境检查

在开始构建前，建议先检查部署环境：

```bash
cd Docker_Tookit
./check-deployment.sh
```

### 2. 构建镜像

从 Docker_Tookit 目录运行构建脚本：

```bash
cd Docker_Tookit
./build-cross-arch.sh
```

这将构建一个 ARM64 架构的 Docker 镜像，可以在 ARM64 设备上运行。

### 3. 快速重建（代码修改后）

如果只是修改了源代码，可以使用快速重建：

```bash
cd Docker_Tookit
./rebuild-code.sh
```

### 4. 运行容器

#### 方式一：使用交互式脚本

```bash
cd Docker_Tookit
./run-docker.sh
```

脚本提供以下选项：
1. 启动 timoo 激光雷达驱动
2. 启动 lidar_target_localization 节点（目标检测）
3. 启动 lidar_reflector_target_tracker 节点（目标跟踪）
4. 启动 heading_estimation 节点
5. 同时启动所有节点（timoo + localization + tracker + heading）
6. 进入容器 bash 交互模式
7. 使用 Docker Compose 启动所有服务

#### 方式二：使用 Docker Compose

```bash
cd Docker_Tookit
docker-compose up -d
```

查看日志：
```bash
docker-compose logs -f
```

停止服务：
```bash
docker-compose down
```

### 5. 手动运行

```bash
# 进入 Docker_Tookit 目录
cd Docker_Tookit

# 运行单个服务
docker run --rm -it \
    --network host \
    --privileged \
    -v ../data:/ros_ws/data \
    -e TIMOO_IP=192.168.188.115 \
    lidar-heading-app:latest \
    bash -c "source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/install/setup.bash && roslaunch timoo_pointcloud TM16.launch"
```

## 配置说明

### 代理配置

如果需要使用代理，请修改以下文件中的代理地址：

- `build-cross-arch.sh` 中的 HTTP_PROXY 和 HTTPS_PROXY
- `buildkitd.toml` 中的 DNS 配置

### 环境变量

在 `docker-compose.yml` 中可以配置：

- `TIMOO_IP`: timoo 激光雷达的 IP 地址（默认: 192.168.188.115）
- `ROS_MASTER_URI`: ROS Master URI
- `ROS_IP`: ROS IP 地址

### 数据目录

数据通过卷挂载映射到容器：
- `../data`: 项目根目录的 data 文件夹映射到容器内的 `/ros_ws/data`

## 路径说明

由于 Docker_Tookit 目录位于项目根目录下，所有路径引用已相应调整：

- Docker 构建上下文：项目根目录（`..`）
- 源代码路径：`../lidar_target_ws/lidar_target01/src`, `../lidar_target_ws/lidar_target02/src/lidar_target_localization`, `../heading_ws/src`, `../timoo/src`
- 数据卷挂载：`../data`

## 注意事项

1. **脚本执行权限**：所有 `.sh` 脚本已设置可执行权限，如果遇到权限问题，运行：
   ```bash
   chmod +x *.sh
   ```

2. **跨平台构建**：本工具包支持从 x86_64 系统构建 ARM64 架构的镜像

3. **依赖项**：确保已安装 Docker 和 Docker Buildx

4. **网络模式**：使用 `--network host` 模式，容器与主机共享网络

5. **特权模式**：使用 `--privileged` 模式以支持硬件访问
