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

    rviz = LaunchConfiguration('rviz')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz with the dual-robot config'
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
                    multi_lica_output_dir],
        output="screen",
    )


    drone_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch_path),
        launch_arguments={
            'body_namespace': 'drone',
            'world_namespace': 'rover',
            'namespace': 'drone',
            'rviz': 'false',
        }.items(),
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
                }.items(),
            )
        ],
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
        }],
        output='screen',
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_rviz_cmd,
        rviz_node,
        cloud_saver_node,
        drone_mapping,
        rover_mapping,
        merge_map_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=cloud_saver_node,
                on_exit=[multi_lica],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=multi_lica,
                on_exit=[camera_init_from_raw_lidar],
            )
        ),
    ])
