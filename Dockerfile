# =========================
# Single-stage: Build and run GTSAM + ROS (minimal, no GUI tools)
# =========================
FROM arm64v8/ros:melodic-ros-base

ENV TZ=Asia/Shanghai
RUN apt-get update && apt-get install -y tzdata && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone

# 安装所有编译和运行依赖（不含 GUI 相关包）
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget unzip pkg-config \
    python-rosdep python-catkin-tools \
    libboost-all-dev libeigen3-dev libomp-dev \
    libpcl-dev ros-melodic-pcl-ros ros-melodic-pcl-conversions \
    ros-melodic-roscpp ros-melodic-rospy ros-melodic-sensor-msgs \
    ros-melodic-geometry-msgs ros-melodic-std-msgs ros-melodic-message-generation ros-melodic-message-runtime \
    ros-melodic-cv-bridge ros-melodic-rosbag \
    libpcap-dev \
    ros-melodic-nodelet \
    ros-melodic-dynamic-reconfigure \
    ros-melodic-image-transport \
    ros-melodic-tf ros-melodic-tf2-ros ros-melodic-tf2-geometry-msgs \
    ros-melodic-diagnostic-updater \
    ros-melodic-angles \
    libyaml-cpp-dev \
    ros-melodic-visualization-msgs \
    ros-melodic-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# 配置 rosdep 源（国内源加速）
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    echo "yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/base.yaml" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update -v || true

# 拷贝 GTSAM 源码并编译安装
COPY gtsam/gtsam-4.0.0-alpha2 /tmp/gtsam
RUN mkdir -p /tmp/gtsam/build && cd /tmp/gtsam/build && \
    cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    make -j4 && make install

# 拷贝工作空间源码
COPY . /root/workspace
WORKDIR /root/workspace

# 自动加载 ROS 和 workspace
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "for ws in /root/workspace/*/devel/setup.bash; do" >> ~/.bashrc && \
    echo "    if [ -f \$ws ]; then source \$ws; fi" >> ~/.bashrc && \
    echo "done" >> ~/.bashrc

# 默认启动 bash
CMD ["bash"]

