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
    libignition-gazebo6-dev \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root

RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
    cd PX4-Autopilot/ && \
    make px4_sitl

WORKDIR /root

RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install && \
    sudo ldconfig /usr/local/lib/RU

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

WORKDIR /root/ros2_ws

RUN colcon build --packages-ignore livox_ros_driver2 && \
    source install/setup.bash

CMD ["bash"]