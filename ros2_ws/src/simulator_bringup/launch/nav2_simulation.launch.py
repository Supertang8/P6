from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
import os

def generate_launch_description():
    # Path to leo_gz_bringup launch file
    leo_gz_pkg = get_package_share_directory('leo_gz_bringup')
    leo_gz_launch = os.path.join(leo_gz_pkg, 'launch', 'leo_gz.launch.py')

    rviz_config = os.path.expanduser('~/ros2_ws/src/simulator_bringup/rviz/Simulator.rviz')

    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
    nav2_params = os.path.expanduser('~/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml')
    nav2_map = os.path.expanduser('~/ros2_ws/src/Leorover/leo_simulator-ros2/leo_gz_worlds/World_maps/leo_p6.yaml')

    return LaunchDescription([


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0','0','0','0','0','0','map','odom']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_odom',
            arguments=['0','0','0','0','0','0','odom','leo_rover/odom']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['-0.1','0','-0.26','0','0','0','leo_rover/base_footprint','base_footprint']
        ),


        # Launch leo_gz
        TimerAction(
            period=2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(leo_gz_launch),
                    ),
                ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
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
