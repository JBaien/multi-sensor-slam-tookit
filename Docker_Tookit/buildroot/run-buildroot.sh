#!/bin/bash

# ============================================
# Run Docker Container on Buildroot ARM64 Systems
# 在 buildroot arm64 系统上运行 Docker 容器
# ============================================

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 配置变量
IMAGE_NAME="lidar-heading-buildroot"
IMAGE_TAG="latest"
CONTAINER_NAME="lidar-heading-buildroot"

# 默认环境变量
TIMOO_IP="${TIMOO_IP:-192.168.188.115}"
ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
ROS_IP="${ROS_IP:-127.0.0.1}"

# 数据卷挂载
DATA_DIR="${DATA_DIR:-./data}"
DATA_MOUNT="${DATA_MOUNT:-/ros_ws/data}"

# 显示菜单
show_menu() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Buildroot ARM64 Docker 容器管理${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${BLUE}当前配置:${NC}"
    echo -e "  镜像: ${IMAGE_NAME}:${IMAGE_TAG}"
    echo -e "  容器名: ${CONTAINER_NAME}"
    echo -e "  TIMOO IP: ${TIMOO_IP}"
    echo -e "  ROS_MASTER_URI: ${ROS_MASTER_URI}"
    echo -e "  ROS_IP: ${ROS_IP}"
    echo -e "  数据目录: ${DATA_DIR} -> ${DATA_MOUNT}"
    echo ""
    echo -e "${YELLOW}请选择操作:${NC}"
    echo "  1) 启动所有节点（timoo + localization + tracker + heading）"
    echo "  2) 仅启动 timoo 激光雷达驱动"
    echo "  3) 仅启动 lidar_target_localization 节点"
    echo "  4) 仅启动 lidar_reflector_target_tracker 节点"
    echo "  5) 仅启动 heading_estimation 节点"
    echo "  6) 进入容器 bash 交互模式"
    echo "  7) 查看容器日志"
    echo "  8) 停止容器"
    echo "  9) 删除容器"
    echo "  0) 退出"
    echo ""
    read -p "请输入选项 [0-9]: " choice
}

# 检查镜像是否存在
check_image() {
    if ! docker images | grep -q "^${IMAGE_NAME}.*${IMAGE_TAG}"; then
        echo -e "${RED}错误: 镜像 ${IMAGE_NAME}:${IMAGE_TAG} 不存在！${NC}"
        echo ""
        echo -e "${YELLOW}请先构建镜像:${NC}"
        echo "  ./build-buildroot.sh"
        echo ""
        echo -e "${YELLOW}或从 tar 文件加载:${NC}"
        echo "  docker load -i ${IMAGE_NAME}-buildroot.tar"
        echo ""
        exit 1
    fi
}

# 启动容器（基础函数）
run_container() {
    local cmd="$1"
    local name_suffix="${2:-}"
    
    local container_name="${CONTAINER_NAME}${name_suffix}"
    
    # 检查容器是否已存在
    if docker ps -a --format '{{.Names}}' | grep -q "^${container_name}$"; then
        echo -e "${YELLOW}容器 ${container_name} 已存在${NC}"
        read -p "是否删除并重新创建? [y/N]: " recreate
        if [[ "$recreate" =~ ^[Yy]$ ]]; then
            docker rm -f ${container_name} 2>/dev/null || true
        else
            echo -e "${YELLOW}启动现有容器...${NC}"
            docker start ${container_name} 2>/dev/null || true
            return
        fi
    fi
    
    echo -e "${GREEN}启动容器 ${container_name}...${NC}"
    
    # 创建数据目录
    mkdir -p ${DATA_DIR}
    
    # 运行容器
    docker run -d \
        --name ${container_name} \
        --network host \
        --privileged \
        -e TIMOO_IP=${TIMOO_IP} \
        -e ROS_MASTER_URI=${ROS_MASTER_URI} \
        -e ROS_IP=${ROS_IP} \
        -v ${DATA_DIR}:${DATA_MOUNT} \
        -v /dev:/dev \
        ${IMAGE_NAME}:${IMAGE_TAG} \
        ${cmd}
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}容器启动成功！${NC}"
        echo -e "${YELLOW}查看日志: docker logs -f ${container_name}${NC}"
    else
        echo -e "${RED}容器启动失败！${NC}"
        exit 1
    fi
}

# 交互式模式
run_interactive() {
    echo -e "${GREEN}启动交互式容器...${NC}"
    
    # 创建数据目录
    mkdir -p ${DATA_DIR}
    
    docker run --rm -it \
        --name ${CONTAINER_NAME}-interactive \
        --network host \
        --privileged \
        -e TIMOO_IP=${TIMOO_IP} \
        -e ROS_MASTER_URI=${ROS_MASTER_URI} \
        -e ROS_IP=${ROS_IP} \
        -v ${DATA_DIR}:${DATA_MOUNT} \
        -v /dev:/dev \
        ${IMAGE_NAME}:${IMAGE_TAG} \
        bash
}

# 查看日志
view_logs() {
    echo -e "${GREEN}查看容器日志...${NC}"
    echo -e "${YELLOW}按 Ctrl+C 退出${NC}"
    echo ""
    docker logs -f ${CONTAINER_NAME}
}

# 停止容器
stop_container() {
    echo -e "${GREEN}停止容器...${NC}"
    docker stop ${CONTAINER_NAME} 2>/dev/null || echo -e "${YELLOW}容器未运行${NC}"
}

# 删除容器
remove_container() {
    echo -e "${GREEN}删除容器...${NC}"
    stop_container
    docker rm ${CONTAINER_NAME} 2>/dev/null || echo -e "${YELLOW}容器不存在${NC}"
}

# 主循环
main() {
    check_image
    
    # 如果有命令行参数，直接执行
    if [ $# -gt 0 ]; then
        case "$1" in
            all)
                run_container "/start_nodes.sh"
                ;;
            timoo)
                run_container "bash -c 'source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/install/setup.bash && roslaunch timoo_pointcloud TM16.launch'" "-timoo"
                ;;
            localization)
                run_container "bash -c 'source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/install/setup.bash && roslaunch lidar_target_localization target_localization.launch'" "-localization"
                ;;
            tracker)
                run_container "bash -c 'source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/install/setup.bash && roslaunch lidar_reflector_target_tracker tracker.launch'" "-tracker"
                ;;
            heading)
                run_container "bash -c 'source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/install/setup.bash && roslaunch heading_estimation heading_estimation.launch'" "-heading"
                ;;
            bash|interactive)
                run_interactive
                ;;
            logs)
                view_logs
                ;;
            stop)
                stop_container
                ;;
            rm|remove)
                remove_container
                ;;
            *)
                echo -e "${RED}未知参数: $1${NC}"
                echo "使用方法: $0 [all|timoo|localization|tracker|heading|bash|logs|stop|rm]"
                exit 1
                ;;
        esac
        exit 0
    fi
    
    # 交互式菜单
    while true; do
        show_menu
        
        case $choice in
            1)
                run_container "/start_nodes.sh"
                ;;
            2)
                run_container "bash -c 'source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/install/setup.bash && roslaunch timoo_pointcloud TM16.launch'" "-timoo"
                ;;
            3)
                run_container "bash -c 'source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/install/setup.bash && roslaunch lidar_target_localization target_localization.launch'" "-localization"
                ;;
            4)
                run_container "bash -c 'source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/install/setup.bash && roslaunch lidar_reflector_target_tracker tracker.launch'" "-tracker"
                ;;
            5)
                run_container "bash -c 'source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/install/setup.bash && roslaunch heading_estimation heading_estimation.launch'" "-heading"
                ;;
            6)
                run_interactive
                ;;
            7)
                view_logs
                ;;
            8)
                stop_container
                ;;
            9)
                remove_container
                ;;
            0)
                echo -e "${GREEN}退出${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}无效选项，请重新选择${NC}"
                ;;
        esac
        
        echo ""
        read -p "按 Enter 继续..."
    done
}

main "$@"
