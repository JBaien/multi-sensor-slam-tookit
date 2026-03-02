#!/bin/bash
# ============================================
# 快速重建脚本（仅重新编译代码）
# 适用于源代码修改后的快速重建
# ============================================

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}快速重建（仅代码层）${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}注意: 此命令会清除Docker构建缓存${NC}"
echo -e "${YELLOW}仅重新编译源代码层，保留基础镜像${NC}"
echo ""

# 清理旧容器和构建缓存
docker stop $(docker ps -aq) 2>/dev/null || true
docker system prune -f

# 使用--no-cache重建代码层
docker buildx build \
    --platform linux/arm64 \
    --file "$(pwd)/Dockerfile.cross-arch" \
    --tag lidar-heading-app:latest \
    --load \
    --progress=plain \
    --no-cache \
    "$(pwd)/.."

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}重建完成！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "可以立即启动容器: ./run-docker.sh"
fi
