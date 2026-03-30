from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the local single-lidar aggregator and livox driver on one machine."""

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic', default_value='/livox/lidar_192_168_10_198')
    aggregated_topic_arg = DeclareLaunchArgument(
        'aggregated_topic', default_value='/rover/aggregated_pointcloud')
    trigger_service_arg = DeclareLaunchArgument(
        'trigger_service', default_value='/rover/trigger_accumulation')
    output_frame_id_arg = DeclareLaunchArgument(
        'output_frame_id', default_value='rover_lidar')
    messages_to_accumulate_arg = DeclareLaunchArgument(
        'messages_to_accumulate', default_value='10')
    downsample_leaf_size_arg = DeclareLaunchArgument(
        'downsample_leaf_size', default_value='0.05')
    
    # Get the path to livox_ros_driver2 package
    livox_package_path = FindPackageShare('livox_ros_driver2')
    livox_launch_file = PythonLaunchDescriptionSource([
        livox_package_path, '/launch/msg_MID360_rover_launch.py'
    ])
    
    # Pointcloud aggregator node
    aggregator_node = Node(
        package='system_package',
        executable='pointcloud_aggregator_node',
        name='pointcloud_aggregator_node',
        output='screen',
        parameters=[
            {'lidar_topic': LaunchConfiguration('lidar_topic')},
            {'aggregated_topic': LaunchConfiguration('aggregated_topic')},
            {'trigger_service': LaunchConfiguration('trigger_service')},
            {'output_frame_id': LaunchConfiguration('output_frame_id')},
            {'messages_to_accumulate': LaunchConfiguration('messages_to_accumulate')},
            {'downsample_leaf_size': LaunchConfiguration('downsample_leaf_size')},
        ],
    )

    # Create launch description with aggregator and livox driver
    return LaunchDescription([
        lidar_topic_arg,
        aggregated_topic_arg,
        trigger_service_arg,
        output_frame_id_arg,
        messages_to_accumulate_arg,
        downsample_leaf_size_arg,

        # Include livox driver launch
        IncludeLaunchDescription(livox_launch_file),
        
        # Launch aggregator node
        aggregator_node,
    ])
