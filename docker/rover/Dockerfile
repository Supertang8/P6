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
        libeigen3-dev \
    	ros-humble-pcl-conversions \
        ros-humble-pcl-msgs \
        ros-humble-pcl-ros \
	    ros-humble-tf2-ros \
	    ros-humble-tf2-eigen \
	    ros-humble-rviz2 \
        ros-humble-ros-gz \
        ros-humble-octomap \
        ros-humble-octomap-msgs \
        ros-humble-octomap-server \
        ros-humble-octomap-rviz-plugins \
        ros-humble-teleop-twist-keyboard \
        ros-humble-bondcpp \
        ros-humble-test-msgs \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "alias cb='cd /root/ros2_ws && colcon build --packages-select livox_ros_driver2 fast_lio system_package mapping_launch livox_converter leo leo_msgs leo_teleop leo_description leo_gz_bringup leo_gz_plugins leo_gz_worlds leo_simulator leo_simulator simulator_bringup mapping_launch nav2_config odom_to_tf_ros2 && colcon build --base-paths src/navigation2 '" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir -p Livox-SDK2/build && \
    cd Livox-SDK2/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

WORKDIR /root/ros2_ws

RUN git clone --depth 1 https://github.com/ros-navigation/navigation2.git --branch $ROS_DISTRO ./src/navigation2 

RUN apt update && \
rosdep install -y \
--from-paths ./src \
--ignore-src \
--skip-keys="gazebo_ros gazebo_ros_pkgs"


WORKDIR /root

COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

WORKDIR /root/ros2_ws

#ENTRYPOINT ["/root/entrypoint.sh"]

CMD ["bash"]


