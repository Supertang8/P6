from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Launch arguments
    world = DeclareLaunchArgument(
        "world",
        default_value="leo_p6",
        description="Gazebo world name",
    )

    # World configuration
    world_config = LaunchConfiguration("world")

    return LaunchDescription([
        world,

        # PX4 SITL + Gazebo
        ExecuteProcess(
            cmd=[
                "bash",
                "-c",
                [
                    "cd /root/PX4-Autopilot-P6 && ",
                    "PX4_GZ_WORLD=",
                    world_config,
                    " make px4_sitl gz_x500_lidar_down",
                ],
            ],
            output="screen",
        ),

        # Micro XRCE-DDS Agent
        ExecuteProcess(
            cmd=[
                "MicroXRCEAgent",
                "udp4",
                "-p",
                "8888",
            ],
            output="screen",
        ),
    ])
