from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
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
        period=4.0,  # wait 4 seconds before starting
        actions=[
            Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                output='screen',
                parameters=[{
    			'frame_id': 'livox_aligned',
			'resolution': 0.1,
    			'base_frame_id': 'body_aligned',

			'filter_speckles': True,

			'filter_ground_plane': True,
			'ground_filter.angle': 0.1,
			'ground_filter.distance': 0.2,
			'ground_filter.plane_distance': 0.5,

			'sensor_model.max_range': 8.0,
			'point_cloud_max_z': 1.2,
			'point_cloud_min_z': 0.1
                }],
                remappings=[
                    ('cloud_in', '/cloud_registered_body')
                ]
            )
        ]
    )

    # Make sure return is inside the function
    return LaunchDescription([
        fastlio,
        octomap
    ])
