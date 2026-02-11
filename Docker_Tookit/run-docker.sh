#!/bin/bash
# ============================================
# Docker容器启动脚本
# 用于快速启动ROS应用容器
# ============================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

IMAGE_NAME="lidar-heading-app:latest"
CONTAINER_NAME="ros-lidar-app"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Docker容器启动脚本${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查镜像是否存在
if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
    echo -e "${RED}错误: 镜像 $IMAGE_NAME 不存在${NC}"
    echo -e "${YELLOW}请先构建镜像: ./build-cross-arch.sh${NC}"
    exit 1
fi

# 启动模式选择
echo ""
echo "请选择启动模式:"
echo "1) 启动timoo激光雷达驱动"
echo "2) 启动lidar_target_detector节点"
echo "3) 启动heading_estimation节点"
echo "4) 同时启动所有节点（timoo + lidar + heading）"
echo "5) 进入容器bash交互模式"
echo "6) 使用Docker Compose启动所有服务"
echo ""

read -p "请输入选项 (1-6): " choice

# 询问是否后台运行
BACKGROUND_MODE=""
if [ "$choice" != "5" ] && [ "$choice" != "6" ]; then
    read -p "是否后台运行? (y/N): " background_answer
    if [ "$background_answer" = "y" ] || [ "$background_answer" = "Y" ]; then
        BACKGROUND_MODE="-d"
        echo -e "${YELLOW}将以后台模式启动容器${NC}"
    else
        BACKGROUND_MODE="--rm -it"
        echo -e "${YELLOW}将以交互模式启动容器${NC}"
    fi
fi

case $choice in
    1)
        echo -e "${YELLOW}启动timoo激光雷达驱动...${NC}"
        read -p "请输入timoo雷达IP地址 [默认: 192.168.188.115]: " timoo_ip
        timoo_ip=${timoo_ip:-192.168.188.115}
        docker run $BACKGROUND_MODE \
            --name "${CONTAINER_NAME}-timoo" \
            --network host \
            --privileged \
            --restart unless-stopped \
            -e LANG=zh_CN.UTF-8 \
            -e LC_ALL=zh_CN.UTF-8 \
            -v "$(dirname "$(pwd)")/data:/ros_ws/data" \
            -e TIMOO_IP="$timoo_ip" \
            "$IMAGE_NAME" \
            bash -c "
                if [ -f /opt/ros/noetic/setup.bash ]; then
                    export LANG=zh_CN.UTF-8
                    export LC_ALL=zh_CN.UTF-8
                    source /opt/ros/noetic/setup.bash &&
                    source /opt/catkin_ws/install/setup.bash &&
                    roslaunch timoo_pointcloud TM16.launch device_ip:=${TIMOO_IP}
                else
                    echo 'ERROR: ROS not found in /opt/ros/noetic/'
                    echo 'Available content:'
                    ls -la /opt/
                    exit 1
                fi
            "
        if [ "$BACKGROUND_MODE" = "-d" ]; then
            echo -e "${GREEN}容器已在后台启动${NC}"
            echo -e "${YELLOW}查看日志: docker logs -f ${CONTAINER_NAME}-timoo${NC}"
            echo -e "${YELLOW}停止容器: docker stop ${CONTAINER_NAME}-timoo${NC}"
        fi
        ;;
    2)
        echo -e "${YELLOW}启动lidar_target_detector节点...${NC}"
        docker run $BACKGROUND_MODE \
            --name "${CONTAINER_NAME}-lidar" \
            --network host \
            --privileged \
            --restart unless-stopped \
            -e LANG=zh_CN.UTF-8 \
            -e LC_ALL=zh_CN.UTF-8 \
            -v "$(pwd)/data:/ros_ws/data" \
            "$IMAGE_NAME" \
            bash -c "
                if [ -f /opt/ros/noetic/setup.bash ]; then
                    export LANG=zh_CN.UTF-8
                    export LC_ALL=zh_CN.UTF-8
                    source /opt/ros/noetic/setup.bash &&
                    source /opt/catkin_ws/install/setup.bash &&
                    roslaunch lidar_target_detection lidar_target_detector.launch
                else
                    echo 'ERROR: ROS not found in /opt/ros/noetic/'
                    echo 'Available content:'
                    ls -la /opt/
                    exit 1
                fi
            "
        if [ "$BACKGROUND_MODE" = "-d" ]; then
            echo -e "${GREEN}容器已在后台启动${NC}"
            echo -e "${YELLOW}查看日志: docker logs -f ${CONTAINER_NAME}-lidar${NC}"
            echo -e "${YELLOW}停止容器: docker stop ${CONTAINER_NAME}-lidar${NC}"
        fi
        ;;
    3)
        echo -e "${YELLOW}启动heading_estimation节点...${NC}"
        docker run $BACKGROUND_MODE \
            --name "${CONTAINER_NAME}-heading" \
            --network host \
            --privileged \
            --restart unless-stopped \
            -e LANG=C.UTF-8 \
            -e LC_ALL=C.UTF-8 \
            -v "$(dirname "$(pwd)")/data:/ros_ws/data" \
            "$IMAGE_NAME" \
            bash -c "
                if [ -f /opt/ros/noetic/setup.bash ]; then
                    export LANG=C.UTF-8
                    export LC_ALL=C.UTF-8
                    source /opt/ros/noetic/setup.bash &&
                    source /opt/catkin_ws/install/setup.bash &&
                    roslaunch heading_estimation heading_estimation.launch
                else
                    echo 'ERROR: ROS not found in /opt/ros/noetic/'
                    echo 'Available content:'
                    ls -la /opt/
                    exit 1
                fi
            "
        if [ "$BACKGROUND_MODE" = "-d" ]; then
            echo -e "${GREEN}容器已在后台启动${NC}"
            echo -e "${YELLOW}查看日志: docker logs -f ${CONTAINER_NAME}-heading${NC}"
            echo -e "${YELLOW}停止容器: docker stop ${CONTAINER_NAME}-heading${NC}"
        fi
        ;;
    4)
        echo -e "${YELLOW}同时启动所有节点...${NC}"
        read -p "请输入timoo雷达IP地址 [默认: 192.168.188.115]: " timoo_ip
        timoo_ip=${timoo_ip:-192.168.188.115}
        docker run $BACKGROUND_MODE \
            --name "$CONTAINER_NAME" \
            --network host \
            --privileged \
            --restart unless-stopped \
            -e LANG=zh_CN.UTF-8 \
            -e LC_ALL=zh_CN.UTF-8 \
            -v "$(pwd)/data:/ros_ws/data" \
            -e TIMOO_IP="$timoo_ip" \
            "$IMAGE_NAME" \
            /start_nodes.sh
        if [ "$BACKGROUND_MODE" = "-d" ]; then
            echo -e "${GREEN}容器已在后台启动${NC}"
            echo -e "${YELLOW}查看日志: docker logs -f ${CONTAINER_NAME}${NC}"
            echo -e "${YELLOW}停止容器: docker stop ${CONTAINER_NAME}${NC}"
        fi
        ;;
    5)
        echo -e "${YELLOW}进入容器交互模式...${NC}"
        docker run --rm -it \
            --name "${CONTAINER_NAME}-shell" \
            --network host \
            --privileged \
            -v "$(dirname "$(pwd)")/data:/ros_ws/data" \
            "$IMAGE_NAME" \
            bash
        ;;
    6)
        echo -e "${YELLOW}使用Docker Compose启动所有服务...${NC}"
        docker-compose up -d
        echo -e "${GREEN}容器已启动，查看日志: docker-compose logs -f${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac
