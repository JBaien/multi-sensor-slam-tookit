#!/bin/bash
# ============================================
# 部署检查脚本
# 功能：检查Docker环境和配置是否满足部署要求
# ============================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Docker部署环境检查${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检查结果数组
CHECKS_PASSED=()
CHECKS_FAILED=()

# 1. 检查系统架构
echo -e "${BLUE}[1/11] 检查系统架构...${NC}"
ARCH=$(uname -m)
echo -e "当前架构: ${YELLOW}$ARCH${NC}"

if [ "$ARCH" = "x86_64" ]; then
    echo -e "${GREEN}✓ 构建环境为x86_64, 可以进行跨架构构建${NC}"
    CHECKS_PASSED+=("系统架构")
elif [ "$ARCH" = "aarch64" ]; then
    echo -e "${GREEN}✓ 当前为ARM64环境, 可以直接运行ARM64镜像${NC}"
    CHECKS_PASSED+=("系统架构")
else
    echo -e "${RED}✗ 不支持的架构: $ARCH${NC}"
    CHECKS_FAILED+=("系统架构")
fi
echo ""

# 2. 检查操作系统
echo -e "${BLUE}[2/11] 检查操作系统...${NC}"
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo -e "操作系统: ${YELLOW}$PRETTY_NAME${NC}"
    echo -e "${GREEN}✓ 操作系统信息已获取${NC}"
    CHECKS_PASSED+=("操作系统")
else
    echo -e "${YELLOW}⚠ 无法确定操作系统版本${NC}"
fi
echo ""

# 3. 检查Docker安装
echo -e "${BLUE}[3/11] 检查Docker安装...${NC}"
if command -v docker &> /dev/null; then
    DOCKER_VERSION=$(docker --version)
    echo -e "Docker版本: ${YELLOW}$DOCKER_VERSION${NC}"
    CHECKS_PASSED+=("Docker安装")
    
    # 检查Docker服务是否运行
    if docker info &> /dev/null; then
        echo -e "${GREEN}✓ Docker服务正在运行${NC}"
    else
        echo -e "${RED}✗ Docker服务未运行，请启动Docker服务${NC}"
        CHECKS_FAILED+=("Docker服务运行")
    fi
else
    echo -e "${RED}✗ Docker未安装${NC}"
    echo -e "请访问 https://docs.docker.com/engine/install/ 安装Docker"
    CHECKS_FAILED+=("Docker安装")
fi
echo ""

# 4. 检查Docker buildx
echo -e "${BLUE}[4/11] 检查Docker buildx...${NC}"
if docker buildx version &> /dev/null; then
    BUILDX_VERSION=$(docker buildx version | head -n1)
    echo -e "Buildx版本: ${YELLOW}$BUILDX_VERSION${NC}"
    echo -e "${GREEN}✓ Docker buildx已安装${NC}"
    CHECKS_PASSED+=("Docker buildx")
else
    echo -e "${RED}✗ Docker buildx未安装或未启用${NC}"
    echo -e "请尝试: docker buildx create --name multiarch --use --bootstrap"
    CHECKS_FAILED+=("Docker buildx")
fi
echo ""

# 5. 检查QEMU
echo -e "${BLUE}[5/11] 检查QEMU模拟器...${NC}"
if docker run --rm --platform linux/arm64 alpine:3.14 uname -m &> /dev/null; then
    echo -e "${GREEN}✓ QEMU模拟器已配置，支持ARM64模拟${NC}"
    CHECKS_PASSED+=("QEMU模拟器")
else
    echo -e "${RED}✗ QEMU模拟器未配置${NC}"
    echo -e "请运行以下命令配置QEMU:"
    echo -e "  docker run --privileged --rm tonistiigi/binfmt --install all"
    CHECKS_FAILED+=("QEMU模拟器")
fi
echo ""

# 6. 检查源代码目录
echo -e "${BLUE}[6/11] 检查源代码目录...${NC}"

# 获取项目根目录
PROJECT_ROOT="$(dirname "$(pwd)")"

# 检查lidar_target_ws/lidar_target01
if [ -d "$PROJECT_ROOT/lidar_target_ws/lidar_target01/src" ]; then
    echo -e "${GREEN}✓ lidar_target_ws/lidar_target01 源代码目录存在${NC}"
    CHECKS_PASSED+=("lidar_target01源码")
else
    echo -e "${RED}✗ lidar_target_ws/lidar_target01/src 目录不存在${NC}"
    CHECKS_FAILED+=("lidar_target01源码")
fi

# 检查lidar_target_ws/lidar_target02
if [ -d "$PROJECT_ROOT/lidar_target_ws/lidar_target02/src/lidar_target_localization" ]; then
    echo -e "${GREEN}✓ lidar_target_ws/lidar_target02 源代码目录存在${NC}"
    CHECKS_PASSED+=("lidar_target02源码")
else
    echo -e "${RED}✗ lidar_target_ws/lidar_target02/src/lidar_target_localization 目录不存在${NC}"
    CHECKS_FAILED+=("lidar_target02源码")
fi

# 检查heading_ws
if [ -d "$PROJECT_ROOT/heading_ws/src" ]; then
    echo -e "${GREEN}✓ heading_ws 源代码目录存在${NC}"
    CHECKS_PASSED+=("heading_ws源码")
else
    echo -e "${RED}✗ heading_ws/src 目录不存在${NC}"
    CHECKS_FAILED+=("heading_ws源码")
fi
echo ""

# 7. 检查Dockerfile
echo -e "${BLUE}[7/10] 检查Dockerfile...${NC}"
if [ -f "Dockerfile.cross-arch" ]; then
    echo -e "${GREEN}✓ Dockerfile.cross-arch 存在${NC}"
    CHECKS_PASSED+=("Dockerfile")
    
    # 检查Dockerfile语法
    if docker pull alpine:latest &> /dev/null; then
        echo "  跳过Dockerfile语法检查（需要构建）"
    fi
else
    echo -e "${RED}✗ Dockerfile.cross-arch 不存在${NC}"
    CHECKS_FAILED+=("Dockerfile")
fi
echo ""

# 8. 检查磁盘空间
echo -e "${BLUE}[8/11] 检查磁盘空间...${NC}"
DISK_USAGE=$(df -h . | awk 'NR==2 {print $5}' | sed 's/%//')
DISK_AVAILABLE=$(df -h . | awk 'NR==2 {print $4}')
echo -e "当前目录磁盘使用率: ${YELLOW}$DISK_USAGE%${NC}"
echo -e "可用空间: ${YELLOW}$DISK_AVAILABLE${NC}"

if [ "$DISK_USAGE" -lt 80 ]; then
    echo -e "${GREEN}✓ 磁盘空间充足${NC}"
    CHECKS_PASSED+=("磁盘空间")
else
    echo -e "${YELLOW}⚠ 磁盘空间不足，建议清理${NC}"
    CHECKS_FAILED+=("磁盘空间")
fi
echo ""

# 9. 检查网络连接
echo -e "${BLUE}[9/11] 检查网络连接...${NC}"
if ping -c 1 -W 2 packages.ros.org &> /dev/null; then
    echo -e "${GREEN}✓ 可以访问ROS软件源${NC}"
    CHECKS_PASSED+=("网络连接(ROS)")
else
    echo -e "${YELLOW}⚠ 无法访问ROS软件源（可能需要配置代理）${NC}"
    CHECKS_FAILED+=("网络连接(ROS)")
fi

if ping -c 1 -W 2 hub.docker.com &> /dev/null; then
    echo -e "${GREEN}✓ 可以访问Docker Hub${NC}"
    CHECKS_PASSED+=("网络连接(Docker)")
else
    echo -e "${YELLOW}⚠ 无法访问Docker Hub（可能需要配置代理）${NC}"
    CHECKS_FAILED+=("网络连接(Docker)")
fi
echo ""

# 10. 检查构建脚本
echo -e "${BLUE}[10/11] 检查构建脚本...${NC}"
if [ -f "$(pwd)/build-cross-arch.sh" ]; then
    if [ -x "$(pwd)/build-cross-arch.sh" ]; then
        echo -e "${GREEN}✓ build-cross-arch.sh 存在且可执行${NC}"
        CHECKS_PASSED+=("构建脚本")
    else
        echo -e "${YELLOW}⚠ build-cross-arch.sh 存在但不可执行${NC}"
        echo -e "请运行: chmod +x build-cross-arch.sh"
        CHECKS_FAILED+=("构建脚本权限")
    fi
else
    echo -e "${RED}✗ build-cross-arch.sh 不存在${NC}"
    CHECKS_FAILED+=("构建脚本")
fi
echo ""

# 汇总结果
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}检查结果汇总${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

echo -e "通过项目: ${GREEN}${#CHECKS_PASSED[@]}${NC}/${#CHECKS_PASSED[@]}${#CHECKS_FAILED[@]}"
echo -e "失败项目: ${RED}${#CHECKS_FAILED[@]}${NC}/${#CHECKS_PASSED[@]}${#CHECKS_FAILED[@]}"
echo ""

if [ ${#CHECKS_PASSED[@]} -gt 0 ]; then
    echo -e "${GREEN}✓ 通过检查:${NC}"
    for item in "${CHECKS_PASSED[@]}"; do
        echo -e "  - $item"
    done
fi

if [ ${#CHECKS_FAILED[@]} -gt 0 ]; then
    echo ""
    echo -e "${RED}✗ 失败/警告:${NC}"
    for item in "${CHECKS_FAILED[@]}"; do
        echo -e "  - $item"
    done
fi

echo ""

# 生成检查报告
REPORT_FILE="deployment_check_$(date +%Y%m%d_%H%M%S).txt"
cat > "$REPORT_FILE" << EOF
Docker部署环境检查报告
生成时间: $(date)
========================================

系统信息:
- 架构: $ARCH
- 操作系统: $PRETTY_NAME
- 磁盘使用率: $DISK_USAGE%
- 可用空间: $DISK_AVAILABLE

检查结果:
- 通过项目 (${#CHECKS_PASSED[@]}):
$(for item in "${CHECKS_PASSED[@]}"; do echo "  - $item"; done)

- 失败/警告 (${#CHECKS_FAILED[@]}):
$(for item in "${CHECKS_FAILED[@]}"; do echo "  - $item"; done)

========================================
EOF

echo -e "检查报告已保存到: ${YELLOW}$REPORT_FILE${NC}"

# 最终建议
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}部署建议${NC}"
echo -e "${GREEN}========================================${NC}"

if [ ${#CHECKS_FAILED[@]} -eq 0 ]; then
    echo -e "${GREEN}✓ 所有检查通过，可以开始构建镜像${NC}"
    echo ""
    echo "运行以下命令开始构建:"
    echo "  ./build-cross-arch.sh"
else
    echo -e "${YELLOW}⚠ 存在一些问题，建议修复后再构建${NC}"
    echo ""
    echo "常见修复命令:"
    if [[ " ${CHECKS_FAILED[@]} " =~ " Docker buildx " ]]; then
        echo "  # 配置Docker buildx"
        echo "  docker buildx create --name multiarch --use --bootstrap"
    fi
    if [[ " ${CHECKS_FAILED[@]} " =~ " QEMU模拟器 " ]]; then
        echo "  # 配置QEMU模拟器"
        echo "  docker run --privileged --rm tonistiigi/binfmt --install all"
    fi
    if [[ " ${CHECKS_FAILED[@]} " =~ " 构建脚本权限 " ]]; then
        echo "  # 添加执行权限"
        echo "  chmod +x build-cross-arch.sh"
    fi
fi

echo ""
