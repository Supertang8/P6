import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapping_launch_path = os.path.join(
        get_package_share_directory('mapping_launch'), 'launch', 'mapping.launch.py')
    rviz_cfg = os.path.join(
        get_package_share_directory('mapping_launch'), 'rviz', 'dual_robot_rviz.rviz')

    # Find the path to the Multi_LiCa src
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
            f"Could not locate Multi_LiCa directory, tried: {multi_lica_src_candidates}")
    calibration_files_dir = os.path.join(multi_lica_src, 'data', 'drone_to_rover_calibration')

    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
    nav2_params = os.path.expanduser('~/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml')


    rviz = LaunchConfiguration('rviz')
    start_rover = LaunchConfiguration('start_rover')
    start_drone = LaunchConfiguration('start_drone')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz with the dual-robot config'
    )

    declare_start_rover_cmd = DeclareLaunchArgument(
        'start_rover',
        default_value='true',
        description='Set to true if runnung on rover or rosbag'
    )

    declare_start_drone_cmd = DeclareLaunchArgument(
        'start_drone',
        default_value='true',
        description='Set to true if runnung on rosbag'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

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

    # MultiLica node runs when cloud_saver_node exits.
    multi_lica_parameter_file = os.path.join(multi_lica_src, 'data', 'multi_LiCa_config.yaml')
    multi_lica_output_dir =os.path.join(get_package_share_directory("multi_lidar_calibrator"), "output")

    multi_lica = Node(
        package="multi_lidar_calibrator",
        executable="multi_lidar_calibrator",
        name="multi_lidar_calibration_node",
        parameters=[multi_lica_parameter_file,
                    multi_lica_output_dir,
                    {'use_sim_time': use_sim_time}],
        output="screen",
    )


    drone_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch_path),
        launch_arguments={
            'body_namespace': 'drone',
            'world_namespace': 'drone',
            'namespace': 'drone',
            'rviz': 'false',
            'use_sim_time': use_sim_time,
            'octomap_resolution': '0.4', # Lower res
            'octomap_model_range': '12.0', # Higher range
        }.items(),
        condition=IfCondition(start_drone),
    )

    rover_mapping = TimerAction(
        # mapping.launch.py starts octomap after 2s; delay rover start to prevent
        # the drone delayed action from resolving with rover namespace.
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch_path),
                launch_arguments={
                    'body_namespace': 'rover',
                    'world_namespace': 'rover',
                    'namespace': 'rover',
                    'rviz': 'false',
                    'use_sim_time': use_sim_time,
                    'octomap_resolution': '0.2', # Higher res
                    'octomap_model_range': '8.0', # Lower range
                }.items(),
            )
        ],
        condition=IfCondition(start_rover),
    )

    camera_init_from_raw_lidar = Node(
        package='mapping_launch',
        executable='camera_init_tf_from_raw_lidar',
        name='camera_init_tf_from_raw_lidar',
        parameters=[{
            'parent_namespace': 'rover',
            'child_namespace': 'drone',
            'odom_topic': 'Odometry',
            'calibration_file': os.path.join(multi_lica_src, 'data', 'calibration_result.txt'),
            'parent_body_to_lidar_xyz': '-0.011,-0.02329,0.04412', # mid360 imu to lidar pos
            'parent_body_to_lidar_xyzw': '0.0,0.0,0.0,1.0', # mid360 imu to lidar rot
            'child_body_to_lidar_xyz': '-0.011,-0.02329,0.04412', # mid360 imu to lidar pos
            'child_body_to_lidar_xyzw': '0.0,0.0,0.0,1.0', # mid360 imu to lidar rot
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
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

    static_lidar_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', ['rover/body'],
            '--child-frame-id', ['livox'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_base_link_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.1', '--y', '0', '--z', '-0.26', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', ['livox'],
            '--child-frame-id', ['base_footprint'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_map_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', ['map'],
            '--child-frame-id', ['rover/camera_init'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_odom_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', ['rover/camera_init'],
            '--child-frame-id', ['odom'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_rviz_cmd,
        declare_start_rover_cmd,
        declare_start_drone_cmd,
        declare_use_sim_time_cmd,
        rviz_node,
        static_lidar_frame,
        static_base_link_frame,
        static_map_frame,
        static_odom_frame,
        rover_mapping,
        drone_mapping,
        merge_map_node,
        map_expander,
    ])
