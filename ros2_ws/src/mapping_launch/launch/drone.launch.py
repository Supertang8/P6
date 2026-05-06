import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_launch_path = os.path.join(
        get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
    livox_drone_launch_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'), 'launch', 'msg_MID360_drone_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')


    #############################################################################
    # Transforms

    # Static TF: drone/camera_init → drone/odom  (fast_lio world frame anchor)
    odom_2_camera_init = Node(
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

    # Dynamic TF: drone/odom → drone/base_link, derived from fast_lio odometry
    odom_2_base_link = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        namespace='drone',
        output='screen',
        parameters=[{
            'odom_topic': 'Odometry',
            'frame_id': 'drone/odom',
            'child_frame_id': 'drone/base_link',
            'use_yaw_only': True,
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

    #############################################################################
    # Nodes

    # Livox MID360 driver for the drone — localhost-only so livox/lidar and
    # livox/imu are not discoverable across the network.
    livox_driver = GroupAction([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_drone_launch_path),
        ),
    ])

    # Pointcloud aggregator starts 10 s after livox to let the LiDAR stabilise.
    # It waits for the rover's gather_aggregated_clouds to call the trigger
    # service before accumulating and then exits.
    aggregator_node = Node(
        package='calibrate_lidars',
        executable='pointcloud_aggregator_node',
        name='pointcloud_aggregator_node',
        namespace='drone',
        output='screen',
        parameters=[
            {'lidar_topic': 'livox/lidar'},
            {'aggregated_topic': 'calibration_pointcloud'},
            {'trigger_service': 'trigger_accumulation'},
            {'output_frame_id': 'drone_lidar'},
            {'messages_to_accumulate': 5},
            {'downsample_leaf_size': 0.05},
            {'min_dist': 1.0},
            {'max_dist': 10.0},
            {'use_sim_time': use_sim_time},
        ],
    )


    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_path),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': 'false',
            'namespace': 'drone',
            'use_sim_time': use_sim_time,
        }.items(),
    )


    return LaunchDescription([
        declare_use_sim_time_cmd,

        # Transforms
        odom_2_camera_init,
        odom_2_base_link,

        # Nodes
        livox_driver,
        TimerAction(period=10.0, actions=[aggregator_node]),
        TimerAction(period=25.0, actions=[fastlio]),
        #TimerAction(period=30.0, actions=[drone_cloud_republisher]),
        #TimerAction(period=10.0, actions=[aggregator_node]),
        #fastlio_after_calibration,
    ])
