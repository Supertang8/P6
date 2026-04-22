#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Start MicroXRCEAgent
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 &

# Launch ROS2 script
ros2 run drone_controller velocityController &
ros2 run px4_controller velocityToDrone &
ros2 launch livox_ros_driver2 msg_MID360_drone_launch.py &
ros2 launch mapping_launch mapping.launch.py

# Keep the container running
wait
