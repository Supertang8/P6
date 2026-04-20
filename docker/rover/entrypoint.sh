#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

colcon build --packages-select livox_ros_driver2 fast_lio system_package mapping_launch livox_converter leo leo_msgs leo_teleop leo_description leo_gz_bringup leo_gz_plugins leo_gz_worlds leo_simulator leo_simulator simulator_bringup mapping_launch nav2_config odom_to_tf_ros2 
colcon build --base-paths src/navigation2 
ros2 launch mapping_launch rover_start.launch.py

# Keep the container running
wait
