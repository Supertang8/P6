import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapping_launch_path = os.path.join(
        get_package_share_directory('mapping_launch'), 'launch', 'mapping.launch.py')

    rviz = LaunchConfiguration('rviz')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Enable RViz in each mapper instance'
    )

    drone_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch_path),
        launch_arguments={
            'namespace': 'drone',
            'rviz': rviz,
        }.items(),
    )

    rover_mapping = TimerAction(
        # mapping.launch.py starts octomap after 2s; delay rover start to prevent
        # the drone delayed action from resolving with rover namespace.
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_path),
                launch_arguments={
                    'namespace': 'rover',
                    'rviz': rviz,
                }.items(),
            )
        ],
    )

    drone_to_rover_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone_to_rover_camera_init_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'drone/odom',
            '--child-frame-id', 'rover/odom',
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_rviz_cmd,
        drone_mapping,
        rover_mapping,
        drone_to_rover_camera_init,
    ])
