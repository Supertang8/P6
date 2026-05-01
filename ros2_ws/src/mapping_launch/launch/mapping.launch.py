import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fast_lio_launch_path = os.path.join(
        get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
    world_namespace = LaunchConfiguration('world_namespace')
    body_namespace = LaunchConfiguration('body_namespace')
    namespace = LaunchConfiguration('namespace')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    octomap_resolution = LaunchConfiguration('octomap_resolution')
    octomap_model_range = LaunchConfiguration('octomap_model_range')

    declare_world_namespace_cmd = DeclareLaunchArgument(
        'world_namespace', default_value='rover',
        description='ROS namespace used to isolate mapper world frame topic'
    )
    declare_body_namespace_cmd = DeclareLaunchArgument(
        'body_namespace', default_value='drone',
        description='ROS namespace used to isolate mapper body frame topic'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='drone',
        description='ROS namespace used to isolate all mapper topics'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_octomap_resolution_cmd = DeclareLaunchArgument(
        'octomap_resolution', default_value='0.2',
        description='Resolution (in meters) of the octomap voxels. eg. 0.1, 0.2, 0.5'
    )
    declare_octomap_model_range_cmd = DeclareLaunchArgument(
        'octomap_model_range', default_value='10.0',
        description='Maximum range (in meters) of the octomap model. eg. 5.0, 10.0, 20.0'
    )   

    # Generates static tf for 'odom' -> 'camera_init', aligning to gravity.
    odom_2_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
            '--frame-id', [namespace, '/camera_init'],
            '--child-frame-id', [namespace, '/odom'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Generates tf for 'odom' -> 'base_link' based on Odometry from FAST-LIO.
    odom_2_base_link = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        namespace=namespace,
        output='screen',
        parameters=[{
            'odom_topic': 'Odometry',
            'frame_id': [namespace, '/odom'],
            'child_frame_id': [namespace, '/base_link'],
            'use_yaw_only': True,
            'use_sim_time': use_sim_time,
        }],
    )

    # Republish FAST-LIO's cloud_registered (world frame) into a body-positioned,
    # world-aligned frame so octomap_server gets a correct sensor_origin while
    # avoiding body attitude in the body->base_link round-trip used by the
    # ground filter.
    cloud_world_aligned_republisher = Node(
        package='mapping_launch',
        executable='cloud_world_aligned_republisher',
        name='cloud_world_aligned_republisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'odom_topic': 'Odometry',
            'cloud_topic': 'cloud_registered',
            'output_cloud_topic': 'cloud_registered_world_aligned',
            'output_frame': 'sensor_world_aligned',
            'use_sim_time': use_sim_time,
        }],
    )

    # (aggregator node removed — octomap will start directly)

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_path),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': rviz,
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Build octomap inside an OpaqueFunction so the namespace substitutions are
    # resolved while the IncludeLaunchDescription scope is still active. If we
    # built it eagerly with LaunchConfigurations, the OnProcessExit handler
    # would fire after the include scope has been popped and the substitutions
    # would resolve against the wrong (or last-set) values.
    def _register_octomap(context, *args, **kwargs):
        ns = LaunchConfiguration('namespace').perform(context)
        world_ns = LaunchConfiguration('world_namespace').perform(context)
        body_ns = LaunchConfiguration('body_namespace').perform(context)
        use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        resolution = float(LaunchConfiguration('octomap_resolution').perform(context))
        model_range = float(LaunchConfiguration('octomap_model_range').perform(context))

        octomap_node = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            namespace=ns,
            output='screen',
            parameters=[{
                'frame_id': f'{world_ns}/camera_init',
                'resolution': resolution,
                'base_frame_id': f'{body_ns}/base_link',
                'filter_speckles': True,
                'filter_ground_plane': True,
                'ground_filter.angle': 0.3,
                'ground_filter.distance': 0.3,
                'ground_filter.plane_distance': 1.0,
                'sensor_model.max_range': model_range,
                'point_cloud_max_z': 1.0,
                'point_cloud_min_z': -0.5,
                'use_sim_time': use_sim,
            }],
            remappings=[
                ('cloud_in', 'cloud_registered_world_aligned'),
                ('/projected_map', 'map'),
                ('/octomap_binary', 'octomap_binary'),
                ('/octomap_full', 'octomap_full'),
                ('/octomap_point_cloud_centers', 'octomap_point_cloud_centers'),
                ('/occupied_cells_vis_array', 'occupied_cells_vis_array'),
                ('/free_cells_vis_array', 'free_cells_vis_array'),
            ],
        )
        return [
            octomap_node,
        ]

    octomap = OpaqueFunction(function=_register_octomap)

    return LaunchDescription([
        declare_world_namespace_cmd,
        declare_body_namespace_cmd,
        declare_rviz_cmd,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_octomap_resolution_cmd,
        declare_octomap_model_range_cmd,
        odom_2_camera_init,
        odom_2_base_link,
        fastlio,
        cloud_world_aligned_republisher,
        octomap,
    ])
