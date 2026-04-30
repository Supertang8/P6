#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mapping_launch'),
                'launch',
                'mapping.launch.py',
            )
        )
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
            output='screen',
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='drone_controller',
                    executable='velocityControllerCBF',
                    name='velocity_controller_cbf',
                    output='screen',
                ),
                Node(
                    package='px4_controller',
                    executable='velocityToDrone',
                    name='velocity_to_drone',
                    output='screen',
                ),
            ],
        ),
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'source /root/ros2_ws/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_drone_launch.py'
                    ],
                    shell=False,
                    output='screen',
                ),
            ]
        ),
        TimerAction(period=20.0, actions=[mapping_launch]),
    ])