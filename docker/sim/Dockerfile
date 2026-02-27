FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install base tools
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    curl \
    wget \
    nano \
    build-essential \
    ros-humble-ros-gz \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    xterm \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["bash"]