#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Start the unified drone controller launch description
ros2 launch drone_controller drone_controller_launch.py

# Keep the container running
wait
