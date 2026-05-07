import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_launch_path = os.path.join(
        get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
    livox_config_dir = os.path.join(
        get_package_share_directory('livox_ros_driver2'), 'config')

    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_robot_cmd = DeclareLaunchArgument(
        'robot',
        default_value='rover',
        choices=['rover', 'drone'],
        description='Robot type: rover or drone (selects livox config and namespace)')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')

    # Static TF: {robot}/camera_init → {robot}/odom  (fast_lio world frame anchor)
    odom_2_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', [robot, '/camera_init'],
            '--child-frame-id', [robot, '/odom'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Livox MID360 driver — config file selected by robot type
    livox_config_path = PythonExpression([
        f'"{livox_config_dir}/MID360_config_rover.json" if "',
        robot,
        f'" == "rover" else "{livox_config_dir}/MID360_config_drone.json"',
    ])

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        namespace=robot,
        output='screen',
        parameters=[
            {'xfer_format': 1},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 5.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': livox_config_path},
            {'cmdline_input_bd_code': 'livox0000000001'},
            {'timesync_en': 0},
        ],
    )

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_path),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': 'false',
            'namespace': robot,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        declare_robot_cmd,
        declare_use_sim_time_cmd,

        odom_2_camera_init,
        livox_driver,
        TimerAction(period=10.0, actions=[fastlio]),
    ])
