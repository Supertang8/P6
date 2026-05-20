"""Cross-RMW /cmd_vel relay (Cyclone -> FastDDS).

Launches two processes on the same host:
  * `in`  side: Cyclone subscriber on /cmd_vel
  * `out` side: FastDDS publisher of /cmd_vel
bridged over a local UNIX domain socket.

The `in` side starts immediately; the `out` side starts 2 s later so the
socket exists before it dials in.

Override CYCLONEDDS_URI / FASTRTPS_DEFAULT_PROFILES_FILE here if the
defaults below do not match your environment.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_cyclonedds_uri = ','.join([
        'file:///root/cyclone_iface.xml',
        'file://' + os.path.join(
            get_package_share_directory('mapping_launch'),
            'config', 'cyclonedds.xml'),
    ])

    declare_cyclonedds_uri = DeclareLaunchArgument(
        'cyclonedds_uri',
        default_value=default_cyclonedds_uri,
        description='CYCLONEDDS_URI for the Cyclone (in) side; '
                    'comma-separated list of file:// URIs.',
    )
    declare_fastrtps_profile = DeclareLaunchArgument(
        'fastrtps_profile',
        default_value='',
        description='Optional FASTRTPS_DEFAULT_PROFILES_FILE for the out side.',
    )

    in_env = {
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        'CYCLONEDDS_URI': LaunchConfiguration('cyclonedds_uri'),
    }
    out_env = {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': LaunchConfiguration('fastrtps_profile'),
    }

    in_side = ExecuteProcess(
        cmd=['ros2', 'run', 'mapping_launch', 'cmd_vel_relay', '--side=in'],
        additional_env=in_env,
        output='screen',
    )

    out_side = ExecuteProcess(
        cmd=['ros2', 'run', 'mapping_launch', 'cmd_vel_relay', '--side=out'],
        additional_env=out_env,
        output='screen',
    )

    return LaunchDescription([
        declare_cyclonedds_uri,
        declare_fastrtps_profile,
        in_side,
        TimerAction(period=2.0, actions=[out_side]),
    ])
