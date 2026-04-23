#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 launch mapping_launch system.launch.py rviz:=true start_drone:=false
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

# Keep the container running
wait