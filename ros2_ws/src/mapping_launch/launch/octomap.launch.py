from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    static_lidar_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', [namespace, '/body'],
            '--child-frame-id', 'livox',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_base_link_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.1', '--y', '0', '--z', '-0.26',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'livox',
            '--child-frame-id', 'base_footprint',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_map_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'map',
            '--child-frame-id', [namespace, '/camera_init'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_odom_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', [namespace, '/camera_init'],
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

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
            'use_sim_time': use_sim_time,
        }],
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
        parameters=[{'use_sim_time': use_sim_time}],
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        namespace=namespace,
        output='screen',
        parameters=[{
            'frame_id': [namespace, '/camera_init'],
            'resolution': 0.2,
            'base_frame_id': [namespace, '/base_link'],
            'filter_speckles': True,
            'filter_ground_plane': True,
            'ground_filter.angle': 0.3,
            'ground_filter.distance': 0.3,
            'ground_filter.plane_distance': 1.0,
            'sensor_model.max_range': 8.0,
            'point_cloud_max_z': 1.0,
            'point_cloud_min_z': -0.5,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud_in', 'cloud_registered'),
            ('/projected_map', 'map'),
            ('/octomap_binary', 'octomap_binary'),
            ('/octomap_full', 'octomap_full'),
            ('/octomap_point_cloud_centers', 'octomap_point_cloud_centers'),
            ('/occupied_cells_vis_array', 'occupied_cells_vis_array'),
            ('/free_cells_vis_array', 'free_cells_vis_array'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rover'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        static_lidar_frame,
        static_base_link_frame,
        odom_2_camera_init,
        static_map_frame,
        static_odom_frame,
        odom_2_base_link,
        octomap_node,
    ])
