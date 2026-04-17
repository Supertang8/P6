from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch a namespaced single-lidar aggregator instance."""

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rover',
        description='ROS namespace used to isolate topics (for example: rover, drone)',
    )

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='livox/lidar',
        description='Relative LiDAR input topic under the selected namespace',
    )
    aggregated_topic_arg = DeclareLaunchArgument(
        'aggregated_topic',
        default_value='aggregated_pointcloud',
        description='Relative output topic for the accumulated cloud under the selected namespace',
    )
    trigger_service_arg = DeclareLaunchArgument(
        'trigger_service',
        default_value='trigger_accumulation',
        description='Relative service name used to trigger cloud accumulation under the selected namespace',
    )
    output_frame_id_arg = DeclareLaunchArgument(
        'output_frame_id', default_value='rover_lidar')
    messages_to_accumulate_arg = DeclareLaunchArgument(
        'messages_to_accumulate', default_value='10')
    downsample_leaf_size_arg = DeclareLaunchArgument(
        'downsample_leaf_size', default_value='0.05')
    min_dist_arg = DeclareLaunchArgument(
        'min_dist',
        default_value='0.0',
        description='Minimum 3D distance (meters) from the sensor to keep points in the accumulated cloud',
    )
    max_dist_arg = DeclareLaunchArgument(
        'max_dist',
        default_value='100.0',
        description='Maximum 3D distance (meters) from the sensor to keep points in the accumulated cloud',
    )
    
    # Pointcloud aggregator node
    aggregator_node = Node(
        package='system_package',
        executable='pointcloud_aggregator_node',
        name='pointcloud_aggregator_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'lidar_topic': LaunchConfiguration('lidar_topic')},
            {'aggregated_topic': LaunchConfiguration('aggregated_topic')},
            {'trigger_service': LaunchConfiguration('trigger_service')},
            {'output_frame_id': LaunchConfiguration('output_frame_id')},
            {'messages_to_accumulate': LaunchConfiguration('messages_to_accumulate')},
            {'downsample_leaf_size': LaunchConfiguration('downsample_leaf_size')},
            {'min_dist': LaunchConfiguration('min_dist')},
            {'max_dist': LaunchConfiguration('max_dist')},
        ],
    )

    # Create launch description for the namespaced aggregator
    return LaunchDescription([
        namespace_arg,
        lidar_topic_arg,
        aggregated_topic_arg,
        trigger_service_arg,
        output_frame_id_arg,
        messages_to_accumulate_arg,
        downsample_leaf_size_arg,
        min_dist_arg,
        max_dist_arg,
        aggregator_node,
    ])
