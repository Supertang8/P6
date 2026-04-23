#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

colcon build --packages-select livox_ros_driver2 fast_lio calibrate_lidars mapping_launch livox_converter leo leo_msgs leo_teleop leo_description leo_gz_bringup leo_gz_plugins leo_gz_worlds leo_simulator leo_simulator simulator_bringup mapping_launch nav2_config odom_to_tf_ros2 merge_map
colcon build --symlink-install --packages-up-to multi_lidar_calibrator --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --base-paths src/navigation2 --packages-ignore nav2_system_tests

ros2 launch livox_ros_driver2 msg_MID360_rover_launch.py &
ros2 launch mapping_launch system.launch.py rviz:=true start_drone:=false &
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:="/root/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml" &

# Keep the container running
wait