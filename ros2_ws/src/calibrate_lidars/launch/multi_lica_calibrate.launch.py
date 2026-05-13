import os

from launch import LaunchDescription
from launch_ros.actions import Node


def _find_multi_lica_src():
    launch_dir = os.path.dirname(__file__)
    candidates = [
        os.path.abspath(os.path.join(launch_dir, '..', '..', 'Multi_LiCa')),
        os.path.abspath(os.path.join(launch_dir, '..', '..', '..', '..', '..', 'src', 'Multi_LiCa')),
    ]
    return next((p for p in candidates if os.path.isdir(p)), candidates[0])


def generate_launch_description():
    multi_lica_src = _find_multi_lica_src()
    parameter_file = os.path.join(multi_lica_src, 'data', 'multi_LiCa_config.yaml')

    multi_lica_node = Node(
        package='multi_lidar_calibrator',
        executable='multi_lidar_calibrator',
        name='multi_lidar_calibration_node',
        output='screen',
        parameters=[parameter_file],
    )

    return LaunchDescription([multi_lica_node])
