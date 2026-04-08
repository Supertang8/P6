import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetLaunchConfiguration, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sanitize_topic_suffix(raw: str) -> str:
    cleaned = []
    for c in raw:
        if c.isalnum() or c == '_':
            cleaned.append(c)
        elif c in ['.', ':', '-', '/']:
            cleaned.append('_')
    return ''.join(cleaned)


def _prepare_ip_address(context):
    ip_address = LaunchConfiguration('ip_address').perform(context)
    return [SetLaunchConfiguration('sanitized_ip_address', _sanitize_topic_suffix(ip_address))]


def _build_launch_actions(context, fast_lio_launch_path):
    sanitized_ip_address = LaunchConfiguration('sanitized_ip_address').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)

    # Generates static tf for 'camera_init'- to 'map'-frame, aligning it to gravity
    odom_2_camera_init = ExecuteProcess(
        cmd=[
        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
	'--x', '0', '--y', '0', '--z', '0', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
	#'--x', '0', '--y', '0', '--z', '0', '--qx', '-0.99578', '--qy', '-0.0', '--qz', '-0.09178', '--qw', '0.00196', # Gravity transform calculated from static rosbag
        '--frame-id', 'odom_' + sanitized_ip_address,
        '--child-frame-id', 'camera_init_' + sanitized_ip_address,
        ],
    )

    # Generates tf for 'odom'- to 'base_link'-frame based on /Odometry msg published by Fast-LIO
    odom_2_base_link = ExecuteProcess(
	cmd=[
	'ros2', 'run', 'odom_to_tf_ros2', 'odom_to_tf',
	'--ros-args',
	'-p', 'odom_topic:=/Odometry_' + sanitized_ip_address,
	'-p', 'frame_id:=odom_' + sanitized_ip_address,
	'-p', 'child_frame_id:=base_link_' + sanitized_ip_address,
	'-p', 'use_yaw_only:=true'
	],
    )

    # FAST-LIO
    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_path),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': rviz,
            'ip_address': sanitized_ip_address,
        }.items(),
    )

    # Octomap (delayed to avoid TF issues)
    octomap = TimerAction(
        period=2.0,  # wait 2 seconds before starting
        actions=[
            Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                output='screen',
                parameters=[{
			'frame_id': 'odom_' + sanitized_ip_address,
			'resolution': 0.1,
			'base_frame_id': 'base_link_' + sanitized_ip_address,

			'filter_speckles': True,

			'filter_ground_plane': True,
			'ground_filter.angle': 0.1,
			'ground_filter.distance': 0.3,
			'ground_filter.plane_distance': 0.1,

			'sensor_model.max_range': 8.0,
			'point_cloud_max_z': 1.2,
			'point_cloud_min_z': 0.0
                }],
                remappings=[
                    ('cloud_in', '/cloud_registered_body_' + sanitized_ip_address),
                    ('/projected_map', '/project_map_' + sanitized_ip_address),
                    ('/octomap_binary', '/octomap_binary_' + sanitized_ip_address),
                    ('/octomap_full', '/octomap_full_' + sanitized_ip_address),
                    ('/octomap_point_cloud_centers', '/octomap_point_cloud_centers_' + sanitized_ip_address),
                    ('/occupied_cells_vis_array', '/occupied_cells_vis_array_' + sanitized_ip_address),
                    ('/free_cells_vis_array', '/free_cells_vis_array_' + sanitized_ip_address)
                ]
            )
        ]
    )

    return [
        odom_2_camera_init,
        odom_2_base_link,
        fastlio,
        octomap,
    ]

def generate_launch_description():
    fast_lio_launch_path = os.path.join(
        get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')

    declare_ip_address_cmd = DeclareLaunchArgument(
        'ip_address', default_value='',
        description='Optional IP suffix passed through to FAST-LIO'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    prepare_ip_address_cmd = OpaqueFunction(function=_prepare_ip_address)

    # Make sure return is inside the function
    return LaunchDescription([
    declare_ip_address_cmd,
    declare_rviz_cmd,
    prepare_ip_address_cmd,
        OpaqueFunction(function=lambda context: _build_launch_actions(context, fast_lio_launch_path))
    ])
