import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapping_launch_path = os.path.join(
        get_package_share_directory('mapping_launch'), 'launch', 'mapping.launch.py')

    rviz = LaunchConfiguration('rviz')
    lidar_tf_xyz = LaunchConfiguration('lidar_tf_xyz')
    lidar_tf_xyzw = LaunchConfiguration('lidar_tf_xyzw')
    rover_body_to_lidar_xyz = LaunchConfiguration('rover_body_to_lidar_xyz')
    rover_body_to_lidar_xyzw = LaunchConfiguration('rover_body_to_lidar_xyzw')
    drone_body_to_lidar_xyz = LaunchConfiguration('drone_body_to_lidar_xyz')
    drone_body_to_lidar_xyzw = LaunchConfiguration('drone_body_to_lidar_xyzw')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Enable RViz in each mapper instance'
    )
    declare_lidar_tf_xyz_cmd = DeclareLaunchArgument(
        'lidar_tf_xyz',
        default_value='0.10320066,-0.01712104,-0.195483',
        description='Raw LiDAR transform translation (rover lidar -> drone lidar), comma-separated xyz'
    )
    declare_lidar_tf_xyzw_cmd = DeclareLaunchArgument(
        'lidar_tf_xyzw',
        default_value='-0.0807881,-0.0742717,0.7194024,0.6858696',
        description='Raw LiDAR transform rotation (rover lidar -> drone lidar), comma-separated xyzw'
    )
    declare_rover_body_to_lidar_xyz_cmd = DeclareLaunchArgument(
        'rover_body_to_lidar_xyz',
        default_value='-0.011,-0.02329,0.04412',
        description='Rover body->lidar translation (FAST_LIO mapping.extrinsic_T), comma-separated xyz'
    )
    declare_rover_body_to_lidar_xyzw_cmd = DeclareLaunchArgument(
        'rover_body_to_lidar_xyzw',
        default_value='0.0,0.0,0.0,1.0',
        description='Rover body->lidar rotation from mapping.extrinsic_R, represented as xyzw'
    )
    declare_drone_body_to_lidar_xyz_cmd = DeclareLaunchArgument(
        'drone_body_to_lidar_xyz',
        default_value='-0.011,-0.02329,0.04412',
        description='Drone body->lidar translation (FAST_LIO mapping.extrinsic_T), comma-separated xyz'
    )
    declare_drone_body_to_lidar_xyzw_cmd = DeclareLaunchArgument(
        'drone_body_to_lidar_xyzw',
        default_value='0.0,0.0,0.0,1.0',
        description='Drone body->lidar rotation from mapping.extrinsic_R, represented as xyzw'
    )

    drone_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch_path),
        launch_arguments={
            'namespace': 'drone',
            'rviz': rviz,
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
                    'namespace': 'rover',
                    'rviz': rviz,
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
            'raw_lidar_parent_to_child_xyz': lidar_tf_xyz,
            'raw_lidar_parent_to_child_xyzw': lidar_tf_xyzw,
            'parent_body_to_lidar_xyz': rover_body_to_lidar_xyz,
            'parent_body_to_lidar_xyzw': rover_body_to_lidar_xyzw,
            'child_body_to_lidar_xyz': drone_body_to_lidar_xyz,
            'child_body_to_lidar_xyzw': drone_body_to_lidar_xyzw,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_rviz_cmd,
        declare_lidar_tf_xyz_cmd,
        declare_lidar_tf_xyzw_cmd,
        declare_rover_body_to_lidar_xyz_cmd,
        declare_rover_body_to_lidar_xyzw_cmd,
        declare_drone_body_to_lidar_xyz_cmd,
        declare_drone_body_to_lidar_xyzw_cmd,
        drone_mapping,
        rover_mapping,
        camera_init_from_raw_lidar,
    ])
