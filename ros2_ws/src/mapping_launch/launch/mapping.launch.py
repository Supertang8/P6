from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # Generates static tf for 'camera_init'- to 'map'-frame, aligning it to gravity
    odom_2_camera_init = ExecuteProcess(
        cmd=[
        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
	'--x', '0', '--y', '0', '--z', '0', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
	#'--x', '0', '--y', '0', '--z', '0', '--qx', '-0.99578', '--qy', '-0.0', '--qz', '-0.09178', '--qw', '0.00196', # Gravity transform calculated from static rosbag
        '--frame-id', 'odom',
        '--child-frame-id', 'camera_init'
        ],
    )

    # Generates tf for 'odom'- to 'base_link'-frame based on /Odometry msg published by Fast-LIO
    odom_2_base_link = ExecuteProcess(
	cmd=[
	'ros2', 'run', 'odom_to_tf_ros2', 'odom_to_tf',
	'--ros-args',
  	'-p', 'odom_topic:=/Odometry',
	'-p', 'frame_id:=odom',
  	'-p', 'child_frame_id:=base_link',
	'-p', 'use_yaw_only:=true'
	],
    )

    # Frame-transformation of body frame to align with gravity
    body_aligned = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0', '--qx', '0.99578', '--qy', '0.0', '--qz', '0.09178', '--qw', '0.00196',
            '--frame-id', 'body',
            '--child-frame-id', 'body_aligned'
            ],
    )

    # FAST-LIO
    fastlio = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'fast_lio', 'mapping.launch.py',
            'config_file:=mid360.yaml',
            'rviz:=true'
        ],
        output='screen'
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
    			'frame_id': 'odom',
			'resolution': 0.1,
    			'base_frame_id': 'base_link',

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
                    ('cloud_in', '/cloud_registered_body')
                ]
            )
        ]
    )

    # Make sure return is inside the function
    return LaunchDescription([
	odom_2_camera_init,
	odom_2_base_link,
	body_aligned,
        fastlio,
        octomap
    ])
