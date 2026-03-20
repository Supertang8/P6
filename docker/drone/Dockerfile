FROM ros:humble-ros-base-jammy
#FROM osrf/ros:humble-desktop-full-jammy

# Install useful dev tools
RUN apt update && \
    # apt upgrade -y && \
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt install -y \
    	python3-colcon-common-extensions \
    	python3-rosdep \
		iputils-ping \
		net-tools \
    	nano \
    	cmake \
    	libpcl-dev \
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
RUN echo "source /root/ros2_drone_ws/install/setup.bash" >> /root/.bashrc 

# Colcon build
# WORKDIR /root/ros2_ws
# RUN  colcon build

WORKDIR /root
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir -p Livox-SDK2/build && \
    cd Livox-SDK2/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

#Remove list
RUN rm -rf /var/lib/apt/lists/* 

WORKDIR /root/ros2_drone_ws

CMD ["bash"]
