#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Start MicroXRCEAgent
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 &

# Launch ROS2 script
ros2 run px4_ros_com offboard_control_pos &
ros2 launch livox_ros_driver2 msg_MID360_launch_drone.py

# Keep the container running
wait
