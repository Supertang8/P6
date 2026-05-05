from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        description='Robot namespace (e.g. drone or rover)',
    )
    declare_messages_cmd = DeclareLaunchArgument(
        'messages_to_accumulate', default_value='5',
        description='Number of LiDAR messages to accumulate per cloud',
    )
    declare_min_dist_cmd = DeclareLaunchArgument(
        'min_dist', default_value='1.0',
        description='Minimum point distance in metres',
    )
    declare_max_dist_cmd = DeclareLaunchArgument(
        'max_dist', default_value='10.0',
        description='Maximum point distance in metres',
    )
    declare_leaf_cmd = DeclareLaunchArgument(
        'downsample_leaf_size', default_value='0.05',
        description='Voxel grid leaf size in metres',
    )

    aggregator_node = Node(
        package='calibrate_lidars',
        executable='pointcloud_aggregator_node',
        name='pointcloud_aggregator_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'lidar_topic': 'livox/lidar',
            'aggregated_topic': 'calibration_pointcloud',
            'trigger_service': 'trigger_accumulation',
            'output_frame_id': [namespace, '_lidar'],
            'messages_to_accumulate': LaunchConfiguration('messages_to_accumulate'),
            'downsample_leaf_size': LaunchConfiguration('downsample_leaf_size'),
            'min_dist': LaunchConfiguration('min_dist'),
            'max_dist': LaunchConfiguration('max_dist'),
        }],
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_messages_cmd,
        declare_min_dist_cmd,
        declare_max_dist_cmd,
        declare_leaf_cmd,
        aggregator_node,
    ])
