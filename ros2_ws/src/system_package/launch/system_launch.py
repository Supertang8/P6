from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import math


def transform_from_xyz_rxyz(x, y, z, rotx_deg, roty_deg, rotz_deg):
    """Build a 4x4 transform from translation and XYZ Euler rotations (degrees)."""
    rx = math.radians(rotx_deg)
    ry = math.radians(roty_deg)
    rz = math.radians(rotz_deg)

    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    # Rotation order: Rx * Ry * Rz (extrinsic XYZ convention)
    r00 = cy * cz
    r01 = -cy * sz
    r02 = sy

    r10 = sx * sy * cz + cx * sz
    r11 = -sx * sy * sz + cx * cz
    r12 = -sx * cy

    r20 = -cx * sy * cz + sx * sz
    r21 = cx * sy * sz + sx * cz
    r22 = cx * cy

    return [
        r00, r01, r02, x,
        r10, r11, r12, y,
        r20, r21, r22, z,
        0.0, 0.0, 0.0, 1.0,
    ]


def generate_launch_description():
    """Launch rover/drone aggregators and the system node in one launch script."""
    messages_to_accumulate_arg = DeclareLaunchArgument(
        'messages_to_accumulate', default_value='10')
    downsample_leaf_size_arg = DeclareLaunchArgument(
        'downsample_leaf_size', default_value='0.05')
    min_dist_arg = DeclareLaunchArgument(
        'min_dist', default_value='1.0')
    max_dist_arg = DeclareLaunchArgument(
        'max_dist', default_value='10.0')

    rover_aggregator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('system_package'),
                'launch',
                'find_tf_launch.py',
            ])
        ),
        launch_arguments={
            'namespace': 'rover',
            'output_frame_id': 'rover_lidar',
            'messages_to_accumulate': LaunchConfiguration('messages_to_accumulate'),
            'downsample_leaf_size': LaunchConfiguration('downsample_leaf_size'),
            'min_dist': LaunchConfiguration('min_dist'),
            'max_dist': LaunchConfiguration('max_dist'),
        }.items(),
    )

    drone_aggregator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('system_package'),
                'launch',
                'find_tf_launch.py',
            ])
        ),
        launch_arguments={
            'namespace': 'drone',
            'output_frame_id': 'drone_lidar',
            'messages_to_accumulate': LaunchConfiguration('messages_to_accumulate'),
            'downsample_leaf_size': LaunchConfiguration('downsample_leaf_size'),
            'min_dist': LaunchConfiguration('min_dist'),
            'max_dist': LaunchConfiguration('max_dist'),
        }.items(),
    )

    # Initial guess in meters and degrees.
    x, y, z = 0.0, 0.0, -0.2
    rotx, roty, rotz = 0.0, -10.0, 90.0

    initial_guess_drone_to_rover = transform_from_xyz_rxyz(
        x=x,
        y=y,
        z=z,
        rotx_deg=rotx,
        roty_deg=roty,
        rotz_deg=rotz,
    )
    
    system_node = Node(
        package='system_package',
        executable='system_node',
        name='system_node',
        output='screen',
        parameters=[
            {'drone_aggregated_topic': '/drone/aggregated_pointcloud'},
            {'rover_aggregated_topic': '/rover/aggregated_pointcloud'},
            {'drone_trigger_service': '/drone/trigger_accumulation'},
            {'rover_trigger_service': '/rover/trigger_accumulation'},
            {'initial_guess_drone_to_rover': initial_guess_drone_to_rover},
        ],
    )

    return LaunchDescription([
        messages_to_accumulate_arg,
        downsample_leaf_size_arg,
        min_dist_arg,
        max_dist_arg,
        rover_aggregator,
        drone_aggregator,
        system_node,
    ])
