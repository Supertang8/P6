from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    fast_lio_launch_path = os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')

    # Packages
    leo_gz_pkg = get_package_share_directory("leo_gz_bringup")
    nav2_pkg = get_package_share_directory("nav2_bringup")

    # Launch files
    leo_gz_launch = os.path.join(
        leo_gz_pkg,
        "launch",
        "leo_gz.launch.py"
    )

    nav2_launch = os.path.join(
        nav2_pkg,
        "launch",
        "navigation_launch.py"
    )

    # Nav2 params
    nav2_params = os.path.expanduser(
        "~/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml"
    )

    # Launch arguments
    world = DeclareLaunchArgument(
        "world",
        default_value="leo_p6",
        description="Gazebo world name",
    )

    # World configuration
    world_config = LaunchConfiguration("world")

    # Nav2
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "params_file": nav2_params,
            "use_sim_time": "true",
        }.items(),
    )

    # Rover Gazebo
    rover_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(leo_gz_launch),
        launch_arguments={
            "sim_world": world_config,
            "robot_ns": "rover",
        }.items(),
    )

    map_expander_node = Node(
            package='livox_converter',
            executable='map_expander',
            name='map_expander',
        )
    
    rover_fastLio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_path),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': 'true',
            'namespace': 'rover',
            'use_sim_time': 'true',
        }.items(),
    )

    rover_cloud_world_aligned_republisher = Node(
        package='mapping_launch',
        executable='cloud_world_aligned_republisher',
        name='cloud_world_aligned_republisher',
        namespace='rover',
        output='screen',
        parameters=[{
            'odom_topic': 'Odometry',
            'cloud_topic': 'cloud_registered',
            'output_cloud_topic': 'cloud_registered_world_aligned',
            'output_frame': 'sensor_world_aligned',
            'use_sim_time': True,
        }],
    )

    rover_octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        namespace='rover',
        output='screen',
        parameters=[{
            'frame_id': 'rover/camera_init',
            'resolution': 0.2,
            'base_frame_id': 'rover/base_link',
            'filter_speckles': True,
            'filter_ground_plane': True,
            'ground_filter.angle': 0.3,
            'ground_filter.distance': 0.3,
            'ground_filter.plane_distance': 1.0,
            'sensor_model.max_range': 8.0,
            'point_cloud_max_z': 1.0,
            'point_cloud_min_z': -0.5,
            'use_sim_time': True,
        }],
        remappings=[
            ('cloud_in', 'cloud_registered_world_aligned'),
            ('/projected_map', 'map'),
            ('/octomap_binary', 'octomap_binary'),
            ('/octomap_full', 'octomap_full'),
            ('/octomap_point_cloud_centers', 'octomap_point_cloud_centers'),
            ('/occupied_cells_vis_array', 'occupied_cells_vis_array'),
            ('/free_cells_vis_array', 'free_cells_vis_array'),
        ],
    )

    drone_fastLio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_path),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': 'false',
            'namespace': 'drone',
            'use_sim_time': 'true',
        }.items(),
    )

    drone_cloud_world_aligned_republisher = Node(
        package='mapping_launch',
        executable='cloud_world_aligned_republisher',
        name='cloud_world_aligned_republisher',
        namespace='drone',
        output='screen',
        parameters=[{
            'odom_topic': 'Odometry',
            'cloud_topic': 'cloud_registered',
            'output_cloud_topic': 'cloud_registered_world_aligned',
            'output_frame': 'sensor_world_aligned',
            'use_sim_time': True,
        }],
    )

    drone_octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        namespace='drone',
        output='screen',
        parameters=[{
            'frame_id': 'rover/camera_init',
            'resolution': 0.2,
            'base_frame_id': 'drone/base_link',
            'filter_speckles': True,
            'filter_ground_plane': True,
            'ground_filter.angle': 0.3,
            'ground_filter.distance': 0.3,
            'ground_filter.plane_distance': 1.0,
            'sensor_model.max_range': 8.0,
            'point_cloud_max_z': 1.0,
            'point_cloud_min_z': -0.5,
            'use_sim_time': True,
        }],
        remappings=[
            ('cloud_in', 'cloud_registered_world_aligned'),
            ('/projected_map', 'map'),
            ('/octomap_binary', 'octomap_binary'),
            ('/octomap_full', 'octomap_full'),
            ('/octomap_point_cloud_centers', 'octomap_point_cloud_centers'),
            ('/occupied_cells_vis_array', 'occupied_cells_vis_array'),
            ('/free_cells_vis_array', 'free_cells_vis_array'),
        ],
    )

    merge_map_node = Node(
        package='merge_map',
        executable='merge_map',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/map1', '/rover/projected_map'),
            ('/map2', '/drone/projected_map'),
        ],
    )

    drone_imu_node = Node(
        package='livox_converter',
        executable='drone_imu',
        name='drone_livox_imu',
        namespace='drone',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    drone_lidar_node = Node(
        package='livox_converter',
        executable='pc2_to_livox',
        name='drone_livox_lidar',
        namespace='drone',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init',
            arguments=['0','0','0','0','0','0','map','rover/camera_init']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['0','0','0','0','0','0','rover/body','rover/livox']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['-0.1','0','-0.26','0','0','0','rover/livox','rover/base_footprint']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_init_to_odom',
            arguments=['0','0','0','0','0','0','rover/camera_init','odom']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_link',
            arguments=['0','0','0','0','0','0','rover/base_link','base_link']
        ),

        rover_gazebo_node,
        rover_fastLio_node,
        rover_cloud_world_aligned_republisher,
        rover_octomap_node,

        drone_imu_node,
        drone_lidar_node,

        drone_fastLio_node,
        drone_cloud_world_aligned_republisher,
        drone_octomap_node,
        

        merge_map_node,
        map_expander_node,

        nav2_node,
    ])