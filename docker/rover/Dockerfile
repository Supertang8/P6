FROM ros:humble-ros-base-jammy
#FROM osrf/ros:humble-desktop-full-jammy

# Install useful dev tools
RUN apt update && \
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt update && \
    apt install -y \
    	python3-colcon-common-extensions \
    	python3-rosdep \
	iputils-ping \
	net-tools \
    	nano \
    	cmake \
    	libpcl-dev \
    	ros-humble-pcl-conversions ros-humble-pcl-msgs ros-humble-pcl-ros \
    	libeigen3-dev \
	ros-humble-tf2-ros \
	ros-humble-tf2-eigen \
	ros-humble-rviz2 \
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

WORKDIR /root/deps_ws/src

RUN git clone https://github.com/Supertang8/livox_ros_driver2.git 

RUN git clone https://github.com/hku-mars/FAST_LIO.git -b ROS2 && \
    cd FAST_LIO && \
    git submodule update --init --recursive

WORKDIR /root/deps_ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"
    
RUN echo "source /root/deps_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["bash"]
