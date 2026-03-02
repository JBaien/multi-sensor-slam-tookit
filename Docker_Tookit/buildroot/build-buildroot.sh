#!/bin/bash

# ============================================
# Build Docker Image for Buildroot ARM64 Systems
# 专为 buildroot arm64 系统构建 Docker 镜像
# ============================================

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 配置变量
IMAGE_NAME="lidar-heading-buildroot"
IMAGE_TAG="latest"
DOCKERFILE_PATH="Dockerfile.buildroot"
BUILD_CONTEXT="../.."  # 构建上下文：项目根目录
REGISTRY=""  # 如果需要推送到仓库，设置为 "your-registry.com/"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Buildroot ARM64 Docker 镜像构建脚本${NC}"
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
      --buildkitd-config "$(pwd)/../buildkitd.toml" \
      --driver-opt network=host \
      --driver-opt env.http_proxy=http://192.168.117.1:7890 \
      --driver-opt env.https_proxy=http://192.168.117.1:7890 \
      --driver-opt env.HTTP_PROXY=http://192.168.117.1:7890 \
      --driver-opt env.HTTPS_PROXY=http://192.168.117.1:7890 || true
else
    docker buildx use multiarch || docker buildx create --name multiarch --driver docker-container --use --bootstrap \
      --buildkitd-config "$(pwd)/../buildkitd.toml" \
      --driver-opt network=host \
      --driver-opt env.http_proxy=http://192.168.117.1:7890 \
      --driver-opt env.https_proxy=http://192.168.117.1:7890 \
      --driver-opt env.HTTP_PROXY=http://192.168.117.1:7890 \
      --driver-opt env.HTTPS_PROXY=http://192.168.117.1:7890
fi

# 设置代理
export HTTP_PROXY=http://192.168.117.1:7890
export HTTPS_PROXY=http://192.168.117.1:7890
export NO_PROXY=localhost,127.0.0.1

echo -e "${YELLOW}代理配置: $HTTP_PROXY${NC}"
echo ""

# 开始构建
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}开始构建镜像${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}镜像名称:${NC} ${REGISTRY}${IMAGE_NAME}:${IMAGE_TAG}"
echo -e "${GREEN}目标架构:${NC} linux/arm64"
echo -e "${GREEN}Dockerfile:${NC} ${DOCKERFILE_PATH}"
echo ""

# 构建参数
BUILD_ARGS="--platform linux/arm64"
BUILD_ARGS="${BUILD_ARGS} --build-arg HTTP_PROXY=${HTTP_PROXY}"
BUILD_ARGS="${BUILD_ARGS} --build-arg HTTPS_PROXY=${HTTPS_PROXY}"
BUILD_ARGS="${BUILD_ARGS} --build-arg NO_PROXY=${NO_PROXY}"
BUILD_ARGS="${BUILD_ARGS} --progress=plain"
BUILD_ARGS="${BUILD_ARGS} -f ${DOCKERFILE_PATH}"

# 执行构建
docker buildx build ${BUILD_ARGS} -t ${REGISTRY}${IMAGE_NAME}:${IMAGE_TAG} ${BUILD_CONTEXT}

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}构建成功！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}镜像信息:${NC}"
    docker images | grep ${IMAGE_NAME}
    echo ""
    echo -e "${YELLOW}使用方法:${NC}"
    echo "1. 保存为 tar 文件（用于传输到 buildroot 设备）:"
    echo "   docker save ${REGISTRY}${IMAGE_NAME}:${IMAGE_TAG} -o ${IMAGE_NAME}-buildroot.tar"
    echo ""
    echo "2. 在 buildroot 设备上加载:"
    echo "   docker load -i ${IMAGE_NAME}-buildroot.tar"
    echo ""
    echo "3. 运行容器:"
    echo "   docker run --rm -it --network host --privileged \\"
    echo "       -e TIMOO_IP=192.168.188.115 \\"
    echo "       ${REGISTRY}${IMAGE_NAME}:${IMAGE_TAG}"
    echo ""
else
    echo -e "${RED}构建失败！${NC}"
    exit 1
fi
