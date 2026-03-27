from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the system_node with configurable LiDAR topics."""
    
    system_node = Node(
        package='system_package',
        executable='system_node',
        name='system_node',
        output='screen',
        parameters=[
            {'drone_lidar_topic': '/livox/lidar_192_168_1_122'},
            {'drone_imu_topic': '/livox/imu_192_168_1_122'},
            {'rover_lidar_topic': '/livox/lidar_192_168_10_198'},
            {'rover_imu_topic': '/livox/imu_192_168_10_198'},
            {
                'initial_guess_drone_to_rover': [
                    0.984807753, 0.0, -0.173648178, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.173648178, 0.0, 0.984807753, -0.2,
                    0.0, 0.0, 0.0, 1.0,
                ]
            },
        ],
    )

    return LaunchDescription([
        system_node,
    ])
