#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Start MicroXRCEAgent and let the UART bridge stabilise before loading ROS nodes
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 &
sleep 2

# Start controllers after the bridge is ready
ros2 run drone_controller velocityControllerCBF &
ros2 run px4_controller velocityToDrone &
sleep 1

# Give the Livox driver time to connect to the LiDAR hardware before
# mapping starts, so FAST_LIO never sees an empty IMU buffer on first scan
ros2 launch livox_ros_driver2 msg_MID360_drone_launch.py &
sleep 2

ros2 launch mapping_launch mapping.launch.py

# Keep the container running
wait
