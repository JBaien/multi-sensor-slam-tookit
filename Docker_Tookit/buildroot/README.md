# Buildroot ARM64 Docker 部署指南

本目录包含用于在 Buildroot ARM64 嵌入式系统上部署 ROS 应用的 Docker 相关文件。

## 目录结构

```
buildroot/
├── Dockerfile.buildroot              # Buildroot 专用 Dockerfile
├── docker-compose.buildroot.yml       # Docker Compose 配置文件
├── build-buildroot.sh                 # Buildroot 镜像构建脚本
├── run-buildroot.sh                   # Buildroot 容器运行脚本（单容器）
├── run-docker-compose-buildroot.sh    # Buildroot Docker Compose 管理脚本
└── README.md                          # 本文档
```

## Buildroot 适配说明

### Buildroot vs Ubuntu 的区别

| 特性 | Buildroot | Ubuntu |
|------|-----------|---------|
| 包管理器 | 无（静态编译） | apt |
| 库依赖 | 需要静态链接或包含所有依赖 | 动态链接，系统包提供 |
| 系统大小 | 极小（通常 < 100MB） | 较大（通常 > 1GB） |
| 优化程度 | 高度优化 | 通用优化 |

### Dockerfile.buildroot 特性

1. **精简运行时依赖**：仅包含必需的运行时库
2. **静态链接支持**：部分库使用静态链接以减少依赖
3. **编译优化**：针对 ARM64 架构优化编译选项
4. **最小化镜像**：移除开发工具和调试工具

## 快速开始

### 1. 在开发机上构建镜像

```bash
./build-buildroot.sh
```

### 2. 保存镜像为 tar 文件

构建成功后，镜像会自动保存为 `lidar-heading-buildroot.tar`，也可以手动保存：

```bash
docker save lidar-heading-buildroot:latest -o lidar-heading-buildroot.tar
```

### 3. 传输到 Buildroot 设备

```bash
# 使用 scp 传输
scp lidar-heading-buildroot.tar user@buildroot-device:/tmp/
```

### 4. 在 Buildroot 设备上加载镜像

```bash
# 登录到 Buildroot 设备
ssh user@buildroot-device

# 加载镜像
docker load -i /tmp/lidar-heading-buildroot.tar

# 验证镜像
docker images | grep lidar-heading-buildroot
```

### 5. 运行容器

#### 方式一：使用交互式脚本（单容器）

```bash
./run-buildroot.sh all
```

#### 方式二：使用 Docker Compose（推荐）

```bash
./run-docker-compose-buildroot.sh up
```

## 运行方式对比

### 单容器模式 (run-buildroot.sh)
- 所有节点在一个容器内运行
- 资源占用较小
- 适合资源受限的设备
- 故障隔离性较弱

### Docker Compose 模式 (run-docker-compose-buildroot.sh)
- 每个节点独立容器运行
- 故障隔离，单个节点崩溃不影响其他
- 健康检查机制
- 便于监控和维护
- 资源占用稍大

**推荐**: 生产环境使用 Docker Compose 模式

## 配置说明

### 环境变量

| 变量名 | 说明 | 默认值 |
|--------|------|--------|
| `TIMOO_IP` | timoo 激光雷达 IP 地址 | 192.168.188.115 |
| `ROS_MASTER_URI` | ROS Master URI | http://localhost:11311 |
| `ROS_IP` | ROS IP 地址 | 127.0.0.1 |

### 数据卷挂载

- `./data`: 数据目录映射到容器内的 `/ros_ws/data`

## Docker Compose 配置说明

### 服务依赖关系

```
timoo-lidar
    ↓ (依赖)
lidar-localization
    ↓ (依赖)
lidar-tracker
    ↓ (依赖)
heading-estimation
```

### 健康检查

每个服务都配置了健康检查：
- `timoo-lidar`: 检查 `/point_cloud` topic
- `lidar-localization`: 检查 `/target_detection` topic
- `lidar-tracker`: 检查 `/target_tracking` topic
- `heading-estimation`: 检查 `/heading_estimate` topic

### 日志管理

Docker Compose 配置了日志轮转：
- 最大文件大小：10MB
- 保留文件数：3
- 总日志大小限制：30MB

## Buildroot 设备要求

### 必需软件

- Docker 19.03 或更高版本
- 支持 ARM64 架构的内核
- 足够的存储空间（至少 5GB）

### 硬件要求

- CPU: ARM64 (aarch64)
- RAM: 建议 2GB 或更多
- 存储: 至少 5GB 可用空间

### 网络要求

- 能够访问 timoo 激光雷达网络（默认 192.168.188.x）
- 网络接口配置正确

## 特性

- 精简依赖：仅包含必需的运行时库
- 静态链接优化：部分库使用静态链接
- ARM64 优化：编译选项针对 ARM64 架构优化
- 最小化镜像：移除开发工具
- 健康检查：Docker Compose 配置健康检查机制

## 相关文档

- `../ubuntu/README.md`: Ubuntu 系统部署指南
- `../README.md`: 通用 Docker 部署说明
