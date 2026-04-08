from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to leo_gz_bringup launch file
    leo_gz_pkg = get_package_share_directory('leo_gz_bringup')
    leo_gz_launch = os.path.join(leo_gz_pkg, 'launch', 'leo_gz.launch.py')

    # Path to fast_lio launch file
    fast_lio_pkg = get_package_share_directory('fast_lio')
    fast_lio_launch = os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
    fast_lio_config = os.path.expanduser('~/ros2_ws/src/FAST_LIO/config/mid360.yaml')

    return LaunchDescription([
        # Launch leo_gz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(leo_gz_launch),
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
                'ground_filter.distance': 0.2,
                'ground_filter.plane_distance': 0.5,
                'sensor_model.max_range': 8.0
            }],
            remappings=[('cloud_in', '/cloud_registered_body')]
        )
    ])