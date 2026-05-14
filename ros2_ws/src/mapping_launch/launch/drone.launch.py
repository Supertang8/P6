import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── paths ─────────────────────────────────────────────────────────────
    livox_drone_launch_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'), 'launch', 'msg_MID360_drone_launch.py')

    # ── arguments ─────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')

    # ══════════════════════════════════════════════════════════════════════
    # 1. HARDWARE
    # ══════════════════════════════════════════════════════════════════════

    # Livox MID360 driver for the drone
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_drone_launch_path),
    )

    # ══════════════════════════════════════════════════════════════════════
    # 2. CALIBRATION
    # ══════════════════════════════════════════════════════════════════════

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

    drone_lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('lio_sam'), 'launch', 'run.launch.py')),
        launch_arguments={
            'namespace': 'drone',
            'rviz': 'false',
            'use_sim_time': use_sim_time,
        }.items(),
    )


    return LaunchDescription([
        declare_use_sim_time_cmd,

        # Nodes
        livox_driver,
        TimerAction(period=10.0, actions=[aggregator_node]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=aggregator_node,
                on_exit=[TimerAction(period=20.0, actions=[drone_lio_sam])],
            )
        ),
    ])
