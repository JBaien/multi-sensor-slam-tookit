# Ubuntu ARM64 Docker 部署指南

本目录包含用于在 Ubuntu ARM64 系统上部署 ROS 应用的 Docker 相关文件。

## 目录结构

```
ubuntu/
├── Dockerfile.cross-arch      # 跨架构 Dockerfile (x86_64 -> ARM64)
├── docker-compose.yml         # Docker Compose 配置文件
├── build-cross-arch.sh        # 跨架构镜像构建脚本
├── rebuild-code.sh            # 快速重建脚本（仅重新编译代码）
├── run-docker.sh              # Docker 容器启动脚本
└── README.md                  # 本文档
```

## 快速开始

### 1. 环境检查

```bash
./check-deployment.sh
```

### 2. 构建镜像

```bash
./build-cross-arch.sh
```

### 3. 运行容器

#### 方式一：使用交互式脚本

```bash
./run-docker.sh
```

#### 方式二：使用 Docker Compose

```bash
docker-compose up -d
```

### 4. 查看日志

```bash
docker-compose logs -f
```

### 5. 停止服务

```bash
docker-compose down
```

## 配置说明

### 环境变量

在 `docker-compose.yml` 中可以配置：

- `TIMOO_IP`: timoo 激光雷达的 IP 地址（默认: 192.168.188.115）
- `ROS_MASTER_URI`: ROS Master URI
- `ROS_IP`: ROS IP 地址

### 数据目录

数据通过卷挂载映射到容器：
- `../data`: 项目根目录的 data 文件夹映射到容器内的 `/ros_ws/data`

## 特性

- 跨平台构建：支持从 x86_64 系统构建 ARM64 架构的镜像
- 完整依赖：包含完整的 ROS 环境和开发工具
- 可视化支持：包含 X11 挂载，支持可视化工具
- 中文支持：语言环境配置为 `zh_CN.UTF-8`

## 注意事项

1. **脚本执行权限**：所有 `.sh` 脚本已设置可执行权限
2. **跨平台构建**：本工具包支持从 x86_64 系统构建 ARM64 架构的镜像
3. **依赖项**：确保已安装 Docker 和 Docker Buildx
4. **网络模式**：使用 `--network host` 模式，容器与主机共享网络
5. **特权模式**：使用 `--privileged` 模式以支持硬件访问

## 相关文档

- `../buildroot/README.md`: Buildroot 系统部署指南
- `../README.md`: 通用 Docker 部署说明
