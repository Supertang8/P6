"""Rover bringup (CycloneDDS) with the drone octomap pushed to the drone Pi.

Same as rover.cyclone.launch.py except drone_octomap_node is NOT launched
here. The drone runs its own octomap via drone.cyclone.octomap.launch.py
so its deskewed pointcloud stays local; only /drone/projected_map crosses
the network to merge_map_node on the rover.

Pair this file with drone.cyclone.octomap.launch.py.
"""

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
    # ── paths ──────────────────────────────────────────────────────────────
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

    nav2_launch = os.path.join(
        get_package_share_directory('mapping_launch'), 'launch', 'nav2.cyclone.launch.py')
    nav2_params = os.path.expanduser(
        '~/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml')

    cyclonedds_uri = 'file://' + os.path.join(
        get_package_share_directory('mapping_launch'), 'config', 'cyclonedds.xml')

    # ── DDS configuration ─────────────────────────────────────────────────
    # Force CycloneDDS as RMW and point it at the workspace-shared XML so
    # discovery uses unicast peers (drone Pi <-> rover laptop) instead of
    # wifi multicast. Must be set before any node spawns.
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_cyclone_uri = SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_uri)

    # ── arguments ──────────────────────────────────────────────────────────
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

    # rover/base_link → base_footprint  (ties rover base to nav2 base, point x in the opposite direction)
    static_base_link_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.1', '--y', '0', '--z', '-0.26',
            '--qx', '0.0', '--qy', '0.0', '--qz', '1.0', '--qw', '0.0',
            '--frame-id', 'rover/base_link',
            '--child-frame-id', 'base_footprint',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # map → rover/map  (ties the nav2 map frame to the rover world frame)
    static_map_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'map',
            '--child-frame-id', 'rover/map',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # rover/odom → odom  (nav2 odometry anchor)
    static_odom_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'rover/odom',
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_ground_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '-0.26',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', 'rover/map',
            '--child-frame-id', 'ground',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 2. ROVER HARDWARE  – livox only; lio_sam starts after calibration
    # ═══════════════════════════════════════════════════════════════════════

    # Livox MID360 driver for the rover
    rover_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_rover_launch_path),
    )

    rover_lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('lio_sam'), 'launch', 'run.launch.py')),
        launch_arguments={
            'namespace': 'rover',
            'rviz': rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Aggregates rover's raw pointclouds into one for calibration
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


    # ═══════════════════════════════════════════════════════════════════════
    # 5. CALIBRATION CHAIN
    #    gather_aggregated_clouds → MultiLiCa → camera_init_tf
    # ═══════════════════════════════════════════════════════════════════════

    # Collects aggregated clouds and saves as .pcd files for Multi_LiCa
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

    # Finds transfomation between drone and rover lidar
    multi_lica = Node(
        package='multi_lidar_calibrator',
        executable='multi_lidar_calibrator',
        name='multi_lidar_calibration_node',
        parameters=[multi_lica_parameter_file, multi_lica_output_dir,
                    {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Reads the MultiLiCa result and publishes rover/odom → drone/odom.
    # Waits for both robots' LIO-SAM odom→livox_frame TFs before publishing,
    # then signals the aggregators to shut down. The lidar lever-arm is
    # encoded in the URDF (lidar_joint origin).
    drone_to_rover_transform = Node(
        package='mapping_launch',
        executable='camera_init_tf_from_raw_lidar',
        name='camera_init_tf_from_raw_lidar',
        parameters=[{
            'parent_namespace': 'rover',
            'child_namespace': 'drone',
            'calibration_file': os.path.join(multi_lica_src, 'data', 'calibration_result.txt'),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 6. OCTOMAP  – rover only; drone octomap is launched on the drone Pi
    #    by drone.cyclone.octomap.launch.py to keep its pointcloud local.
    # ═══════════════════════════════════════════════════════════════════════

    # Rover octomap starts after calibration (MultiLiCa exits), together
    # with rover LIO-SAM whose data it consumes.
    rover_octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        namespace='rover',
        output='screen',
        parameters=[{
            'frame_id': 'rover/odom',
            'resolution': 0.2,
            # rover/odom is gravity-aligned (LIO-SAM gravity-calibrates) and
            # static-identity with rover/map. Using it keeps the TF lookup
            # at the mapping rate (rover/lidar_link -> rover/odom is broadcast
            # alongside the cloud), avoiding the IMU-rate race that any
            # rover/base_link descendant exposes. Floor sits at z ≈ -0.26
            # in odom, well inside ground_filter.plane_distance.
            'base_frame_id': 'rover/odom',
            'filter_speckles': True,
            'filter_ground_plane': True,
            'ground_filter.angle': 0.3,
            'ground_filter.distance': 0.2,
            'ground_filter.plane_distance': 1.0,
            'sensor_model.max_range': 8.0,
	    #'sensor_model.hit': 0.7,
	    #'sensor_model.miss': 0.4,
	    #'sensor_model.min': 0.12,
	    #'sensor_model.max': 0.97,
	    #'latch': False,
            # Bounds in rover/odom. Floor near z=-0.26; widen to absorb
            # slope drift in odom.
            'point_cloud_min_z': -1.0,
            'point_cloud_max_z': 1.5,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud_in', 'lio_sam/mapping/cloud_deskewed_sync'),
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
        # DDS env vars MUST come before any Node/IncludeLaunchDescription so
        # every spawned process inherits CycloneDDS + the unicast peer config.
        set_rmw,
        set_cyclone_uri,

        # ── launch arguments ────────────────────────────────────────────
        declare_rviz_cmd,
        declare_use_sim_time_cmd,

        # ── static TFs (must be first) ──────────────────────────────────
        static_base_link_frame,
        static_map_frame,
        static_odom_frame,
	static_ground_frame,


        # ── rover hardware (livox + aggregator for calibration) ─────────
        merge_map_node,
        map_expander,
        rover_livox,

        TimerAction(period=10.0, actions=[rover_aggregator]),
        TimerAction(period=10.0, actions=[cloud_saver_node]),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=cloud_saver_node,
                on_exit=[multi_lica],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=multi_lica,
                on_exit=[drone_to_rover_transform],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=multi_lica,
                on_exit=[TimerAction(period=10.0, actions=[rover_lio_sam])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=multi_lica,
                on_exit=[TimerAction(period=30.0, actions=[rover_octomap_node])],
            )
        ),
        #TimerAction(period=70.0, actions=[nav2_node]),
    ])
