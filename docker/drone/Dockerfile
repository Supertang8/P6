FROM ros:humble-ros-base-jammy

# Install useful dev tools
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
	iputils-ping \
	net-tools \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["bash"]
