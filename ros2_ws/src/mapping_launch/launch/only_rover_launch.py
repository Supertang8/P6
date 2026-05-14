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

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    nav2_params = os.path.expanduser(
        '~/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml')

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

    # ═══════════════════════════════════════════════════════════════════════
    # 2. ROVER HARDWARE  – livox driver and lio_sam
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

    # ═══════════════════════════════════════════════════════════════════════
    # 6. OCTOMAP  – rover only
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

    map_expander = Node(
        package='livox_converter',
        executable='map_expander',
        name='map_expander',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/merge_map', '/rover/projected_map'),
        ],
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
        # ── launch arguments ────────────────────────────────────────────
        declare_rviz_cmd,
        declare_use_sim_time_cmd,

        # ── static TFs (must be first) ──────────────────────────────────
        static_base_link_frame,
        static_map_frame,
        static_odom_frame,
        

        # ── rover hardware ─────────
        map_expander,
        rover_livox,
        
        TimerAction(period=10.0, actions=[rover_lio_sam]),
        TimerAction(period=25.0, actions=[rover_octomap_node]),

        #TimerAction(period=70.0, actions=[nav2_node]),
    ])
