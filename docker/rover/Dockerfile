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
        pip \
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

# Copy only package manifests so rosdep can resolve dependencies
COPY ros2_ws/src /tmp/ws/src
WORKDIR /tmp/ws

# Install all ROS dependencies from package.xml files
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y\
    --from-paths ./src \
    --ignore-src \
    --skip-keys="gazebo_ros gazebo_ros_pkgs"

# Clean up (since you'll mount the real workspace at runtime)
RUN rm -rf /tmp/ws

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "alias cb='cd /root/ros2_ws && colcon build --packages-select livox_ros_driver2 fast_lio calibrate_lidars mapping_launch livox_converter leo leo_msgs leo_teleop leo_description leo_gz_bringup leo_gz_plugins leo_gz_worlds leo_simulator leo_simulator simulator_bringup mapping_launch nav2_config odom_to_tf_ros2 merge_map drone_controller && colcon build --symlink-install --packages-up-to multi_lidar_calibrator --cmake-args -DCMAKE_BUILD_TYPE=Release && colcon build --base-paths src/navigation2 --packages-ignore nav2_system_tests'" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Install Multi_Lica for Lidar calibration
WORKDIR /root/ros2_ws/src
RUN git clone --recurse-submodules https://github.com/TUMFTM/Multi_LiCa.git && \
    cd Multi_LiCa/ && \
    pip install --no-cache-dir -r requirements.txt && \
    pip install --ignore-installed --no-cache-dir -r requirements.txt


# Install TEASER plus plus
WORKDIR /root/
RUN git clone https://github.com/MIT-SPARK/TEASER-plusplus.git && \
    pip install --upgrade packaging pip && \
    cd TEASER-plusplus && \
    pip install .


# Install Livox SDK
WORKDIR /root
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir -p Livox-SDK2/build && \
    cd Livox-SDK2/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig


COPY docker/rover/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

WORKDIR /root/ros2_ws

ENTRYPOINT ["/root/entrypoint.sh"]

#CMD ["bash"]



