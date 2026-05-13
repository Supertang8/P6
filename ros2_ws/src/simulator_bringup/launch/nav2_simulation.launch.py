from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    # Path to leo_gz_bringup launch file
    leo_gz_pkg = get_package_share_directory('leo_gz_bringup')
    leo_gz_launch = os.path.join(leo_gz_pkg, 'launch', 'leo_gz.launch.py')

    fast_lio_pkg = get_package_share_directory('fast_lio')
    fast_lio_launch = os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
    fast_lio_config = os.path.expanduser('~/ros2_ws/src/FAST_LIO/config/mid360.yaml')

    rviz_config = os.path.expanduser('~/ros2_ws/src/simulator_bringup/rviz/Simulator_nav2.rviz')

    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
    nav2_params = os.path.expanduser('~/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml')
    nav2_map = os.path.expanduser('~/ros2_ws/src/Leorover/leo_simulator-ros2/leo_gz_worlds/World_maps/leo_p6.yaml')

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value="leo_p6",
        description="Path to the Gazebo world file",
    )

    return LaunchDescription([
        sim_world,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init',
            arguments=['0','0','0','0','0','0','map','camera_init']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['0','0','0','0','0','0','body','livox']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['-0.1','0','-0.26','0','0','0','livox','base_footprint']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_init_to_odom',
            arguments=['0','0','0','0','0','0','camera_init','odom']
        ),

        # Launch leo_gz
        TimerAction(
            period=2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(leo_gz_launch),
                        launch_arguments={
                            'sim_world': LaunchConfiguration("sim_world"),
                        }.items()
                    ),
                ]
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={'config_file': fast_lio_config, 'rviz_cfg': rviz_config}.items()
        ),


        TimerAction(
            period=5.0,
                actions=[

                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav2_launch),
                        launch_arguments={
                            'params_file': nav2_params,
                            'use_sim_time': 'true',
                        }.items()        
                    ),    
                ]
        ),

        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            namespace='',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': nav2_map},
                {'use_sim_time': True}
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),
    ])
"""     
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),  
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'true'
            }.items()     
        ),        """
