#FROM ros:humble-ros-base-jammy
FROM osrf/ros:humble-desktop-full-jammy

# Install useful dev tools
RUN apt update && apt install -y \
    	python3-colcon-common-extensions \
    	python3-rosdep \
	iputils-ping \
	net-tools \
    	nano \
    	cmake \
    	libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir -p Livox-SDK2/build && \
    cd Livox-SDK2/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install
    
RUN apt update && apt install ros-humble-pcl-conversions ros-humble-pcl-ros

WORKDIR /root/ros2_ws

CMD ["bash"]
