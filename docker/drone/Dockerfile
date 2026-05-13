FROM ros:humble-ros-base-jammy

# Install useful dev tools
RUN apt update && \
    # apt upgrade -y && \
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    add-apt-repository ppa:borglab/gtsam-release-4.1 && \
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
        ros-humble-perception-pcl \
        ros-humble-vision-opencv \
        ros-humble-xacro \
        libgtsam-dev libgtsam-unstable-dev \
    	ros-humble-pcl-conversions ros-humble-pcl-msgs ros-humble-pcl-ros
 
#Mavlink communcation
RUN apt update && \
    apt install -y python3-pip  && \
    pip3 install future  && \ 
    pip3 install mavproxy  && \ 
    apt remove -y modemmanager
    
#DDS stuff
WORKDIR /root
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
	cd Micro-XRCE-DDS-Agent && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make && \
	sudo make install && \
	sudo ldconfig /usr/local/lib/

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc 
RUN echo "alias cb='colcon build --packages-select livox_converter calibrate_lidars odom_to_tf_ros2 mapping_launch livox_ros_driver2 px4_msgs px4_ros2_cpp px4_controller drone_controller system_package autorccar_interfaces lio_sam'" >> /root/.bashrc 


# Install Livox SDK
WORKDIR /root
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir -p Livox-SDK2/build 
    
RUN cd Livox-SDK2/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

#Remove list
RUN rm -rf /var/lib/apt/lists/* 

# Copy entrypoint script
#COPY entrypoint.sh /root/entrypoint.sh
#RUN chmod +x /root/entrypoint.sh

WORKDIR /root/ros2_ws

# Set entrypoint
#ENTRYPOINT ["/root/entrypoint.sh"]
