import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fast_lio_launch_path = os.path.join(
        get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
    world_namespace = LaunchConfiguration('world_namespace')
    body_namespace = LaunchConfiguration('body_namespace')
    namespace = LaunchConfiguration('namespace')
    rviz = LaunchConfiguration('rviz')

    declare_world_namespace_cmd = DeclareLaunchArgument(
        'world_namespace', default_value='rover',
        description='ROS namespace used to isolate mapper world frame topic'
    )
    declare_body_namespace_cmd = DeclareLaunchArgument(
        'body_namespace', default_value='drone',
        description='ROS namespace used to isolate mapper body frame topic'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='drone',
        description='ROS namespace used to isolate all mapper topics'
    )

    # Generates static tf for 'odom' -> 'camera_init', aligning to gravity.
    odom_2_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', [namespace, '/camera_init'],
            '--child-frame-id', [namespace, '/odom'],
        ],
    )

    # Generates tf for 'odom' -> 'base_link' based on Odometry from FAST-LIO.
    odom_2_base_link = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        namespace=namespace,
        output='screen',
        parameters=[{
            'odom_topic': 'Odometry',
            'frame_id': [namespace, '/odom'],
            'child_frame_id': [namespace, '/base_link'],
            'use_yaw_only': True,
        }],
    )

    # Pointcloud aggregator node
    aggregator_node = Node(
        package='calibrate_lidars',
        executable='pointcloud_aggregator_node',
        name='pointcloud_aggregator_node',
        namespace=namespace,
        output='screen',
        parameters=[
            {'lidar_topic': 'livox/lidar'},
            {'aggregated_topic': 'calibration_pointcloud'},
            {'trigger_service': 'trigger_accumulation'},
            {'output_frame_id': [namespace, '_lidar']},
            {'messages_to_accumulate': 5},
            {'downsample_leaf_size': 0.05},
            {'min_dist': 1.0},
            {'max_dist': 10.0},
        ],
    )

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_path),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': rviz,
            'namespace': namespace,
        }.items(),
    )

    # Octomap (delayed to avoid TF startup race).
    octomap = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'frame_id': [world_namespace, '/camera_init'],
                    'resolution': 0.2,
                    'base_frame_id': [body_namespace, '/base_link'],
                    'filter_speckles': True,
                    'filter_ground_plane': True,
                    'ground_filter.angle': 0.3,
                    'ground_filter.distance': 0.3,
                    'ground_filter.plane_distance': 1.0,
                    'sensor_model.max_range': 8.0,
                    'point_cloud_max_z': 0.5,
                    'point_cloud_min_z': -0.5,
                }],
                remappings=[
                    ('cloud_in', 'cloud_registered_body'),
                    ('/projected_map', 'map'),
                    ('/octomap_binary', 'octomap_binary'),
                    ('/octomap_full', 'octomap_full'),
                    ('/octomap_point_cloud_centers', 'octomap_point_cloud_centers'),
                    ('/occupied_cells_vis_array', 'occupied_cells_vis_array'),
                    ('/free_cells_vis_array', 'free_cells_vis_array'),
                ],
            )
        ]
    )

    return LaunchDescription([
        declare_world_namespace_cmd,
        declare_body_namespace_cmd,
        declare_rviz_cmd,
        declare_namespace_cmd,
        odom_2_camera_init,
        odom_2_base_link,
        fastlio,
        octomap,
        aggregator_node,
    ])
