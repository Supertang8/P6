FROM ros:humble-ros-base-jammy
#FROM osrf/ros:humble-desktop-full-jammy

# Install useful dev tools
RUN apt update && \
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt install -y \
    	python3-colcon-common-extensions \
    	python3-rosdep \
		iputils-ping \
		net-tools \
    	nano \
    	cmake \
    	wget
 
#Mavlink communcation
RUN apt install -y python3-pip  && \
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
	make install && \
	ldconfig /usr/local/lib/
	
#PX4 Stuff
WORKDIR /root
# --branch v1.15.0 --depth 1 
RUN git clone https://github.com/jacob-slot/PX4-Autopilot-P6.git --recursive && \
	bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
	cd PX4-Autopilot/ && \
	make px4_sitl
	
# RUN apt install -y ros-humble-ros-gzgarden


# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> /root/.bashrc 

#Remove list
RUN rm -rf /var/lib/apt/lists/* 
    

WORKDIR /root/ws

CMD ["bash"]
