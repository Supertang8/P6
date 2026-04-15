from launch import LaunchDescription
from launch_ros.actions import Node
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
    """Launch the system_node with configurable LiDAR topics."""
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
            {'drone_lidar_topic': '/livox/lidar_192_168_1_122'},
            {'drone_imu_topic': '/livox/imu_192_168_1_122'},
            {'rover_lidar_topic': '/livox/lidar_192_168_10_198'},
            {'rover_imu_topic': '/livox/imu_192_168_10_198'},
            {'initial_guess_drone_to_rover': initial_guess_drone_to_rover},
        ],
    )

    return LaunchDescription([
        system_node,
    ])
