import os.path
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prepare_rviz_config(context):
    rviz_cfg_path = LaunchConfiguration('rviz_cfg').perform(context)
    ip_address = LaunchConfiguration('ip_address').perform(context)
    suffix_token = f'_{ip_address}' if ip_address else ''

    with open(rviz_cfg_path, 'r', encoding='utf-8') as src:
        rviz_text = src.read()

    rviz_text = rviz_text.replace('__IP_SUFFIX__', suffix_token)

    fd, generated_path = tempfile.mkstemp(prefix='fastlio_rviz_', suffix='.rviz')
    with os.fdopen(fd, 'w', encoding='utf-8') as dst:
        dst.write(rviz_text)

    return [SetLaunchConfiguration('resolved_rviz_cfg', generated_path)]


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    ip_address = LaunchConfiguration('ip_address')
    rviz_use = LaunchConfiguration('rviz')
    resolved_rviz_cfg = LaunchConfiguration('resolved_rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_ip_address_cmd = DeclareLaunchArgument(
        'ip_address', default_value='',
        description='Optional IP suffix appended to all FAST-LIO sub/pub topic names'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    prepare_rviz_config_cmd = OpaqueFunction(function=_prepare_rviz_config)

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {
                        'use_sim_time': use_sim_time,
                        'common.ip_address': ParameterValue(ip_address, value_type=str),
                    }],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', resolved_rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_ip_address_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(prepare_rviz_config_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
