"""Nav2 launch with composition and staged lifecycle activation.

Based on nav2_bringup/launch/navigation_launch.py. Differences:

  * Always uses composition (single ``component_container_isolated`` process)
    so the 7 nav2 servers share one rclcpp context instead of forking 7
    executables.
  * ``autostart`` is disabled and replaced with three lifecycle_manager
    instances that activate disjoint subsets of the stack, fired one after
    another via TimerAction. This spreads the configure/activate spike
    (costmap rolls, plugin loads, first BT parse) over time instead of
    hitting all at once.

The activation is one-shot: each lifecycle_manager runs ``STARTUP`` once on
its own bond set, then sits idle monitoring.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


# Staged groups. controller+planner first (heavy costmap roll on configure),
# then the recovery/smoothing servers, then the BT/orchestration layer that
# depends on the action servers above being available.
STAGE_1_NODES = ['controller_server', 'planner_server']
STAGE_2_NODES = ['behavior_server', 'smoother_server']
STAGE_3_NODES = ['bt_navigator', 'waypoint_follower', 'velocity_smoother']

STAGE_2_DELAY_SEC = 5.0
STAGE_3_DELAY_SEC = 10.0


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {'use_sim_time': use_sim_time, 'autostart': 'false'}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='Name of the component container for the composed nav2 stack')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
        output='screen')

    load_composable_nodes = LoadComposableNodes(
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_in')]),
        ],
    )

    def _stage_manager(name, node_names):
        return Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=name,
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': node_names}])

    stage_1 = _stage_manager('lifecycle_manager_nav_stage1', STAGE_1_NODES)
    stage_2 = _stage_manager('lifecycle_manager_nav_stage2', STAGE_2_NODES)
    stage_3 = _stage_manager('lifecycle_manager_nav_stage3', STAGE_3_NODES)

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    ld.add_action(stage_1)
    ld.add_action(TimerAction(period=STAGE_2_DELAY_SEC, actions=[stage_2]))
    ld.add_action(TimerAction(period=STAGE_3_DELAY_SEC, actions=[stage_3]))
    return ld
