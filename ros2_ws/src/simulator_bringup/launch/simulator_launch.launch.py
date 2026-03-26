from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
#
def generate_launch_description():

    PX4_DIR = os.environ.get("PX4_AUTOPILOT_PATH", "~/PX4-Autopilot")

    return LaunchDescription([

        # Start PX4 SITL + Gazebo
        ExecuteProcess(
            cmd=[
                "bash", "-c",
                f"cd {PX4_DIR} && PX4_GZ_WORLD=leo_p6 make px4_sitl gz_x500_lidar_front"
            ],
            output="screen"
        ),

        # Start Micro XRCE-DDS Agent (ROS 2 <-> PX4 bridge)
        ExecuteProcess(
            cmd=[
                "MicroXRCEAgent", "udp4", "-p", "8888"
            ],
            output="screen"
        ),

    ])