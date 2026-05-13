import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')
    namespace = LaunchConfiguration('namespace')
    start_rviz = LaunchConfiguration('rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    namespace_declare = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes')

    rviz_declare = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz')

    # Function to create namespaced frame name
    def make_frame_name(frame_name):
        """Prepend namespace to frame name if namespace is not empty"""
        return PythonExpression([
            "('" , namespace, "' + '/' if '", namespace, "' else '') + '", frame_name, "'"
        ])

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        namespace_declare,
        rviz_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                make_frame_name('map'),
                make_frame_name('odom')
            ],
            parameters=[parameter_file],
            namespace=namespace,
            output='screen'
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path, ' namespace:=', namespace])
            }],
            namespace=namespace
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[
                parameter_file,
                {
                    'lidarFrame': make_frame_name('livox_frame'),
                    'baselinkFrame': make_frame_name('base_link'),
                    'odometryFrame': make_frame_name('odom'),
                    'mapFrame': make_frame_name('map'),
                }
            ],
            output='screen',
            namespace=namespace
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[
                parameter_file,
                {
                    'lidarFrame': make_frame_name('livox_frame'),
                    'baselinkFrame': make_frame_name('base_link'),
                    'odometryFrame': make_frame_name('odom'),
                    'mapFrame': make_frame_name('map'),
                }
            ],
            output='screen',
            namespace=namespace
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen',
            namespace=namespace
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[
                parameter_file,
                {
                    'lidarFrame': make_frame_name('livox_frame'),
                    'baselinkFrame': make_frame_name('base_link'),
                    'odometryFrame': make_frame_name('odom'),
                    'mapFrame': make_frame_name('map'),
                }
            ],
            output='screen',
            namespace=namespace
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(start_rviz),
            output='screen',
            namespace=namespace
        )
    ])
