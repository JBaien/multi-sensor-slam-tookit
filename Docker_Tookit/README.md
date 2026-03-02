# Docker 部署工具包

本目录包含用于构建和部署 ROS 应用的 Docker 相关文件，支持 Ubuntu ARM64 和 Buildroot ARM64 两种部署目标。

## 目录结构

```
Docker_Tookit/
├── ubuntu/                    # Ubuntu ARM64 系统部署文件
│   ├── Dockerfile.cross-arch  # 跨架构 Dockerfile
│   ├── docker-compose.yml     # Docker Compose 配置
│   ├── build-cross-arch.sh    # 构建脚本
│   ├── rebuild-code.sh        # 快速重建脚本
│   ├── run-docker.sh          # 运行脚本
│   └── README.md              # Ubuntu 部署说明
│
├── buildroot/                 # Buildroot ARM64 系统部署文件
│   ├── Dockerfile.buildroot   # Buildroot 专用 Dockerfile
│   ├── docker-compose.buildroot.yml  # Docker Compose 配置
│   ├── build-buildroot.sh     # 构建脚本
│   ├── run-buildroot.sh       # 单容器运行脚本
│   ├── run-docker-compose-buildroot.sh  # Compose 管理脚本
│   └── README.md              # Buildroot 部署说明
│
├── check-deployment.sh        # 部署环境检查脚本
├── buildkitd.toml            # Docker Buildx 配置
└── .dockerignore             # Docker 构建忽略文件
```

## 选择合适的部署方案

### Ubuntu ARM64 系统

**适用场景**：
- 标准 Ubuntu Server/Desktop 环境
- 需要可视化工具支持
- 开发和测试环境
- 有充足存储空间（> 1GB）

**使用方法**：
```bash
cd ubuntu
./build-cross-arch.sh      # 构建镜像
./run-docker.sh            # 或使用 docker-compose up -d
```

详细说明见 [ubuntu/README.md](./ubuntu/README.md)

### Buildroot ARM64 系统

**适用场景**：
- Buildroot 嵌入式 Linux 系统
- 资源受限的设备
- 生产环境部署
- 存储空间有限（< 1GB）

**使用方法**：
```bash
cd buildroot
./build-buildroot.sh       # 构建镜像
./run-docker-compose-buildroot.sh up   # 或使用 ./run-buildroot.sh all
```

详细说明见 [buildroot/README.md](./buildroot/README.md)

## 部署方案对比

| 特性 | Ubuntu ARM64 | Buildroot ARM64 |
|------|-------------|-----------------|
| 镜像大小 | 较大 (~3GB) | 较小 (~1.5GB) |
| 运行时依赖 | 完整依赖 | 精简依赖 |
| 可视化支持 | 支持 | 不支持 |
| 语言环境 | zh_CN.UTF-8 | C.UTF-8 |
| 适用系统 | Ubuntu Server/Desktop | Buildroot 嵌入式系统 |
| 资源占用 | 较高 | 较低 |
| 开发工具 | 包含 | 不包含 |
| 部署难度 | 较低 | 中等 |

## 快速参考

### Ubuntu ARM64

```bash
# 构建镜像
cd ubuntu && ./build-cross-arch.sh

# 使用 Docker Compose 运行
docker-compose up -d

# 查看日志
docker-compose logs -f

# 停止服务
docker-compose down
```

### Buildroot ARM64

```bash
# 构建镜像
cd buildroot && ./build-buildroot.sh

# 保存镜像
docker save lidar-heading-buildroot:latest -o lidar-heading-buildroot.tar

# 传输到设备
scp lidar-heading-buildroot.tar user@device:/tmp/

# 在设备上加载并运行
ssh user@device
docker load -i /tmp/lidar-heading-buildroot.tar
./run-docker-compose-buildroot.sh up
```

## 环境要求

### 开发机（构建镜像）

- Docker 19.03+
- Docker Buildx
- 网络连接（下载依赖）
- 足够的磁盘空间（> 10GB）

### 目标设备（运行容器）

- Docker 19.03+
- ARM64 架构
- 足够的内存（建议 2GB+）
- 网络连接（访问激光雷达）

## 常见问题

### Q: 如何选择部署方案？

A: 如果目标设备是 Ubuntu 系统，使用 ubuntu/ 目录下的文件；如果是 Buildroot 系统，使用 buildroot/ 目录下的文件。

### Q: 可以在 x86_64 设备上运行吗？

A: 不可以。这些镜像都是为 ARM64 架构编译的。如果需要在 x86_64 上测试，需要修改 Dockerfile 中的平台参数。

### Q: Buildroot 版本可以可视化吗？

A: 不可以。Buildroot 版本移除了 X11 相关的依赖和挂载，以减小镜像体积。如果需要可视化，请使用 Ubuntu 版本。

### Q: 如何更新代码？

A: 修改源代码后，在开发机重新构建镜像，然后传输到目标设备加载。

## 技术支持

- Ubuntu 部署问题：查看 [ubuntu/README.md](./ubuntu/README.md)
- Buildroot 部署问题：查看 [buildroot/README.md](./buildroot/README.md)
- 通用问题：查看各目录下的 README.md 文档

## 许可证

本项目遵循项目根目录下的许可证。
