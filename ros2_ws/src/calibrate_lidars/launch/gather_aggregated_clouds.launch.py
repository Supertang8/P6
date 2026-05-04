import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_multi_lica_src():
    launch_dir = os.path.dirname(__file__)
    candidates = [
        os.path.abspath(os.path.join(launch_dir, '..', '..', 'Multi_LiCa')),
        os.path.abspath(os.path.join(launch_dir, '..', '..', '..', '..', '..', 'src', 'Multi_LiCa')),
    ]
    return next((p for p in candidates if os.path.isdir(p)), candidates[0])


def generate_launch_description():
    multi_lica_src = _find_multi_lica_src()
    calibration_files_dir = os.path.join(multi_lica_src, 'data', 'drone_to_rover_calibration')

    declare_drone_topic_cmd = DeclareLaunchArgument(
        'drone_aggregated_topic', default_value='/drone/calibration_pointcloud',
        description='Topic where the drone aggregator publishes its cloud',
    )
    declare_rover_topic_cmd = DeclareLaunchArgument(
        'rover_aggregated_topic', default_value='/rover/calibration_pointcloud',
        description='Topic where the rover aggregator publishes its cloud',
    )
    declare_drone_service_cmd = DeclareLaunchArgument(
        'drone_trigger_service', default_value='/drone/trigger_accumulation',
        description='Service to trigger the drone aggregator',
    )
    declare_rover_service_cmd = DeclareLaunchArgument(
        'rover_trigger_service', default_value='/rover/trigger_accumulation',
        description='Service to trigger the rover aggregator',
    )
    declare_drone_output_cmd = DeclareLaunchArgument(
        'drone_output_path',
        default_value=os.path.join(calibration_files_dir, 'lidar_2.pcd'),
        description='Output PCD file path for the drone cloud',
    )
    declare_rover_output_cmd = DeclareLaunchArgument(
        'rover_output_path',
        default_value=os.path.join(calibration_files_dir, 'lidar_1.pcd'),
        description='Output PCD file path for the rover cloud',
    )
    declare_timeout_cmd = DeclareLaunchArgument(
        'request_retry_timeout_sec', default_value='10.0',
        description='Seconds before retrying a failed trigger request',
    )

    gather_node = Node(
        package='calibrate_lidars',
        executable='gather_aggregated_clouds',
        name='gather_aggregated_clouds',
        output='screen',
        parameters=[{
            'drone_aggregated_topic': LaunchConfiguration('drone_aggregated_topic'),
            'rover_aggregated_topic': LaunchConfiguration('rover_aggregated_topic'),
            'drone_trigger_service': LaunchConfiguration('drone_trigger_service'),
            'rover_trigger_service': LaunchConfiguration('rover_trigger_service'),
            'drone_output_path': LaunchConfiguration('drone_output_path'),
            'rover_output_path': LaunchConfiguration('rover_output_path'),
            'request_retry_timeout_sec': LaunchConfiguration('request_retry_timeout_sec'),
        }],
    )

    return LaunchDescription([
        declare_drone_topic_cmd,
        declare_rover_topic_cmd,
        declare_drone_service_cmd,
        declare_rover_service_cmd,
        declare_drone_output_cmd,
        declare_rover_output_cmd,
        declare_timeout_cmd,
        gather_node,
    ])
