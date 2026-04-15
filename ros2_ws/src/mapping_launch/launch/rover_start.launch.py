from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to leo_gz_bringup launch file
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    livox_launch = os.path.join(livox_pkg, 'launch', 'msg_MID360_rover_launch.py')

    # Path to fast_lio launch file
    fast_lio_pkg = get_package_share_directory('fast_lio')
    fast_lio_launch = os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
    fast_lio_config = os.path.expanduser('~/ros2_ws/src/FAST_LIO/config/mid360.yaml')

    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
    nav2_params = os.path.expanduser('~/ros2_ws/src/nav2_config/config/nav2_params_sim.yaml')


    return LaunchDescription([
        # Launch leo_gz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch),
        ),

        # Launch fast_lio with config_file argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={'config_file': fast_lio_config}.items()
        ),

        # Launch octomap_server_node with parameters
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'camera_init',
                'resolution': 0.1,
                'base_frame_id': 'body',
                'filter_speckles': True,
                'filter_ground_plane': True,
                'ground_filter.angle': 0.1,
                'ground_filter.distance': 0.3,
                'ground_filter.plane_distance': 0.8,
                'sensor_model.max_range': 8.0
            }],
            remappings=[('cloud_in', '/cloud_registered_body'), ('projected_map', '/map')]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init',
            arguments=['0','0','0','0','0','0','map','camera_init']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['0','0','0','0','0','0','body','base_footprint']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_init_to_odom',
            arguments=['0','0','0','0','0','0','camera_init','odom']
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),  
        ),
    ])
"""     
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),  
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'true'
            }.items()     
        ),        """
