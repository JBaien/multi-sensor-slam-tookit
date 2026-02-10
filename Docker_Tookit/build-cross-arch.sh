#!/bin/bash
# ============================================
# 跨架构Docker镜像构建脚本
# 功能：从x86_64系统构建aarch64架构的ROS应用镜像
# 访问docker.io 时使用了代理，根据环境更改代理IP和端口
# ============================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 配置变量
IMAGE_NAME="lidar-heading-app"
IMAGE_TAG="latest"
DOCKERFILE="Dockerfile.cross-arch"
REGISTRY=""  # 如果需要推送到仓库，设置为 "your-registry.com/"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}跨架构Docker镜像构建脚本${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查Docker是否安装
if ! command -v docker &> /dev/null; then
    echo -e "${RED}错误: Docker未安装${NC}"
    exit 1
fi

# 检查Docker版本是否支持buildx
DOCKER_VERSION=$(docker version --format '{{.Server.Version}}')
echo -e "${YELLOW}Docker版本: $DOCKER_VERSION${NC}"

# 启用Docker Buildx
echo -e "${YELLOW}正在配置Docker Buildx...${NC}"
if ! docker buildx version &> /dev/null; then
    echo -e "${YELLOW}Buildx不可用, 尝试启用...${NC}"
    docker buildx create --name multiarch --driver docker-container --use --bootstrap \
      --buildkitd-config "$(pwd)/buildkitd.toml" \
      --driver-opt network=host \
      --driver-opt env.http_proxy=http://192.168.117.1:7890 \
      --driver-opt env.https_proxy=http://192.168.117.1:7890 \
      --driver-opt env.HTTP_PROXY=http://192.168.117.1:7890 \
      --driver-opt env.HTTPS_PROXY=http://192.168.117.1:7890 || true
else
    docker buildx use multiarch || docker buildx create --name multiarch --driver docker-container --use --bootstrap \
      --buildkitd-config "$(pwd)/buildkitd.toml" \
      --driver-opt network=host \
      --driver-opt env.http_proxy=http://192.168.117.1:7890 \
      --driver-opt env.https_proxy=http://192.168.117.1:7890 \
      --driver-opt env.HTTP_PROXY=http://192.168.117.1:7890 \
      --driver-opt env.HTTPS_PROXY=http://192.168.117.1:7890
fi

docker buildx inspect --bootstrap

# 显示当前平台信息
echo -e "${YELLOW}当前系统平台: $(uname -m)${NC}"
echo -e "${YELLOW}目标平台: linux/arm64${NC}"

# 构建镜像
FULL_IMAGE_NAME="${REGISTRY}${IMAGE_NAME}:${IMAGE_TAG}"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}开始构建Docker镜像...${NC}"
echo -e "${GREEN}镜像名称: $FULL_IMAGE_NAME${NC}"
echo -e "${GREEN}========================================${NC}"

# 使用buildx进行跨平台构建
export HTTP_PROXY=http://192.168.117.1:7890
export HTTPS_PROXY=http://192.168.117.1:7890
export NO_PROXY=localhost,127.0.0.1

docker buildx build \
    --platform linux/arm64 \
    --file "$(pwd)/$DOCKERFILE" \
    --tag "$FULL_IMAGE_NAME" \
    --load \
    --progress=plain \
    --build-arg HTTP_PROXY=http://192.168.117.1:7890 \
    --build-arg HTTPS_PROXY=http://192.168.117.1:7890 \
    "$(pwd)/.."

if [ $? -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}镜像构建成功！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${YELLOW}镜像信息:${NC}"
    docker images | grep "$IMAGE_NAME"
    
    echo ""
    echo -e "${GREEN}使用方法:${NC}"
    echo -e "${YELLOW}1. 直接运行（交互式）:${NC}"
    echo "  docker run --rm -it $FULL_IMAGE_NAME"
    
    echo ""
    echo -e "${YELLOW}2. 运行并启动所有节点:${NC}"
    echo "  docker run --rm -it $FULL_IMAGE_NAME /start_nodes.sh"
    
    echo ""
    echo -e "${YELLOW}3. 后台运行节点:${NC}"
    echo "  docker run -d --name ros-app $FULL_IMAGE_NAME /start_nodes.sh"
    
    echo ""
    echo -e "${YELLOW}4. 指定ROS参数运行:${NC}"
    echo "  docker run --rm -it \\"
    echo "    -e ROS_MASTER_URI=http://host_ip:11311 \\"
    echo "    -v /path/to/bag:/data \\"
    echo "    $FULL_IMAGE_NAME"
    
    echo ""
    echo -e "${YELLOW}5. 在ARM64设备上运行:${NC}"
    echo "  docker save $FULL_IMAGE_NAME | gzip > ros-app.tar.gz"
    echo "  # 将镜像传输到ARM64设备"
    echo "  docker load < ros-app.tar.gz"
    
    echo ""
    echo -e "${YELLOW}6. 单独启动lidar_target_detector:${NC}"
    echo "  docker run --rm -it $FULL_IMAGE_NAME"
    echo "  # 在容器内执行: roslaunch lidar_target_detection lidar_target_detector.launch"
    
    echo ""
    echo -e "${YELLOW}7. 单独启动heading_estimation:${NC}"
    echo "  docker run --rm -it $FULL_IMAGE_NAME"
    echo "  # 在容器内执行: roslaunch heading_estimation heading_estimation.launch"
    
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}镜像构建失败！${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
