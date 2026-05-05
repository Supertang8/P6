import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── paths ────────────────────────────────────────────────────────────────
    fast_lio_launch_path = os.path.join(
        get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
    livox_rover_launch_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'), 'launch', 'msg_MID360_rover_launch.py')
    rviz_cfg = os.path.join(
        get_package_share_directory('mapping_launch'), 'rviz', 'dual_robot_rviz.rviz')

    launch_dir = os.path.dirname(__file__)
    multi_lica_src_candidates = [
        os.path.abspath(os.path.join(launch_dir, '..', '..', 'Multi_LiCa')),
        os.path.abspath(os.path.join(launch_dir, '..', '..', '..', '..', '..', 'src', 'Multi_LiCa')),
    ]
    multi_lica_src = next(
        (p for p in multi_lica_src_candidates if os.path.isdir(p)),
        multi_lica_src_candidates[0])
    if not os.path.isdir(multi_lica_src):
        raise RuntimeError(
            f'Could not locate Multi_LiCa directory, tried: {multi_lica_src_candidates}')
    calibration_files_dir = os.path.join(multi_lica_src, 'data', 'drone_to_rover_calibration')

    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
    nav2_params = os.path.expanduser('~/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml')

    # ── args ─────────────────────────────────────────────────────────────────
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Start RViz with the dual-robot config')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')

    # ═══════════════════════════════════════════════════════════════════════
    # 1. STATIC TF FRAMES  – published first so everything that follows has
    #    a complete TF tree immediately on startup.
    # ═══════════════════════════════════════════════════════════════════════

    # rover/camera_init → rover/odom  (fast_lio world frame anchor)
    rover_odom_2_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='rover',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'rover/camera_init',
            '--child-frame-id', 'rover/odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # drone/camera_init → drone/odom  (mirrors the drone's own static TF for
    # cases where the rover processes drone data before the drone comes up)
    drone_odom_2_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='drone',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'drone/camera_init',
            '--child-frame-id', 'drone/odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # rover/body → livox  (required by nav2 for the rover footprint)
    static_lidar_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'rover/body',
            '--child-frame-id', 'livox',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # livox → base_footprint  (required by nav2)
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

    # map → rover/camera_init  (ties the nav2 map frame to the rover world frame)
    static_map_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'map',
            '--child-frame-id', 'rover/camera_init',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # rover/camera_init → odom  (nav2 odometry anchor)
    static_odom_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'rover/camera_init',
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 2. ROVER HARDWARE  – livox only; fast_lio starts after calibration
    # ═══════════════════════════════════════════════════════════════════════

    # Livox MID360 driver for the rover
    rover_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_rover_launch_path),
    )

    rover_fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_path),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': 'false',
            'namespace': 'rover',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rover_odom_2_base_link = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        namespace='rover',
        output='screen',
        parameters=[{
            'odom_topic': 'Odometry',
            'frame_id': 'rover/odom',
            'child_frame_id': 'rover/base_link',
            'use_yaw_only': True,
            'use_sim_time': use_sim_time,
        }],
    )


    rover_aggregator = Node(
        package='calibrate_lidars',
        executable='pointcloud_aggregator_node',
        name='pointcloud_aggregator_node',
        namespace='rover',
        output='screen',
        parameters=[
            {'lidar_topic': 'livox/lidar'},
            {'aggregated_topic': 'calibration_pointcloud'},
            {'trigger_service': 'trigger_accumulation'},
            {'output_frame_id': 'rover_lidar'},
            {'messages_to_accumulate': 5},
            {'downsample_leaf_size': 0.05},
            {'min_dist': 1.0},
            {'max_dist': 10.0},
            {'use_sim_time': use_sim_time},
        ],
    )

    rover_cloud_republisher = Node(
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
            'use_sim_time': use_sim_time,
        }],
    )



    drone_cloud_republisher = Node(
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
            'use_sim_time': use_sim_time,
        }],
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 5. CALIBRATION CHAIN
    #    gather_aggregated_clouds → MultiLiCa → camera_init_tf
    # ═══════════════════════════════════════════════════════════════════════

    cloud_saver_node = Node(
        package='calibrate_lidars',
        executable='gather_aggregated_clouds',
        name='gather_aggregated_clouds',
        output='screen',
        parameters=[
            {'drone_aggregated_topic': '/drone/calibration_pointcloud'},
            {'rover_aggregated_topic': '/rover/calibration_pointcloud'},
            {'drone_trigger_service': '/drone/trigger_accumulation'},
            {'rover_trigger_service': '/rover/trigger_accumulation'},
            {'rover_output_path': os.path.join(calibration_files_dir, 'lidar_1.pcd')},
            {'drone_output_path': os.path.join(calibration_files_dir, 'lidar_2.pcd')},
            {'request_retry_timeout_sec': 10.0},
            {'use_sim_time': use_sim_time},
        ],
    )

    multi_lica_parameter_file = os.path.join(multi_lica_src, 'data', 'multi_LiCa_config.yaml')
    multi_lica_output_dir = os.path.join(
        get_package_share_directory('multi_lidar_calibrator'), 'output')

    multi_lica = Node(
        package='multi_lidar_calibrator',
        executable='multi_lidar_calibrator',
        name='multi_lidar_calibration_node',
        parameters=[multi_lica_parameter_file, multi_lica_output_dir,
                    {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Reads the MultiLiCa result and publishes rover/camera_init → drone/camera_init.
    # Waits for both robots' fast_lio TFs before publishing, then signals
    # the aggregators to shut down.
    camera_init_from_raw_lidar = Node(
        package='mapping_launch',
        executable='camera_init_tf_from_raw_lidar',
        name='camera_init_tf_from_raw_lidar',
        parameters=[{
            'parent_namespace': 'rover',
            'child_namespace': 'drone',
            'odom_topic': 'Odometry',
            'calibration_file': os.path.join(multi_lica_src, 'data', 'calibration_result.txt'),
            'parent_body_to_lidar_xyz': '-0.011,-0.02329,0.04412',
            'parent_body_to_lidar_xyzw': '0.0,0.0,0.0,1.0',
            'child_body_to_lidar_xyz': '-0.011,-0.02329,0.04412',
            'child_body_to_lidar_xyzw': '0.0,0.0,0.0,1.0',
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 6. OCTOMAPS  – rover and drone, both built on the rover
    # ═══════════════════════════════════════════════════════════════════════

    # Rover octomap starts after calibration (MultiLiCa exits), together
    # with rover fast_lio whose data it consumes.
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
            'ground_filter.distance': 0.1,
            'ground_filter.plane_distance': 1.0,
            'sensor_model.max_range': 8.0,
            'point_cloud_max_z': 1.0,
            'point_cloud_min_z': -1.0,
            'use_sim_time': use_sim_time,
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

    # Drone octomap and camera_init_tf both start after MultiLiCa exits.
    # The octomap will wait for the inter-robot TF that camera_init_tf
    # publishes once the drone's fast_lio TF becomes available.
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
            'ground_filter.distance': 0.1,
            'ground_filter.plane_distance': 1.0,
            'sensor_model.max_range': 8.0,
            'point_cloud_max_z': 1.0,
            'point_cloud_min_z': -1.0,
            'use_sim_time': use_sim_time,
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

    # ═══════════════════════════════════════════════════════════════════════
    # 7. DOWNSTREAM  – map merge, navigation, visualisation
    # ═══════════════════════════════════════════════════════════════════════

    merge_map_node = Node(
        package='merge_map',
        executable='merge_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/map1', '/rover/projected_map'),
            ('/map2', '/drone/projected_map'),
        ],
    )

    map_expander = Node(
        package='livox_converter',
        executable='map_expander',
        name='map_expander',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_rviz_cmd,
        declare_use_sim_time_cmd,

        # ── static TFs (must be first) ──────────────────────────────────
        rover_odom_2_camera_init,
        drone_odom_2_camera_init,
        rover_odom_2_base_link,
        static_lidar_frame,
        static_base_link_frame,
        static_map_frame,
        static_odom_frame,
        

        # ── rover hardware (livox + aggregator for calibration) ──────────
        merge_map_node,
        map_expander,
        rover_livox,
        TimerAction(period=10.0, actions=[rover_aggregator]),
        TimerAction(period=15.0, actions=[cloud_saver_node]),
        TimerAction(period=20.0, actions=[camera_init_from_raw_lidar]),
        TimerAction(period=25.0, actions=[rover_fastlio]),
        TimerAction(period=35.0, actions=[rover_cloud_republisher, drone_cloud_republisher]),
        TimerAction(period=45.0, actions=[rover_octomap_node, drone_octomap_node]),
        #TimerAction(period=60.0, actions=[nav2_node]),

    ])
