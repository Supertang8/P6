"""Drone bringup (CycloneDDS) with drone-side octomap.

Same as drone.cyclone.launch.py but additionally runs the drone's
octomap_server on the drone Pi instead of on the rover laptop. This keeps
the high-rate deskewed pointcloud local to the drone — only the projected
2D map (/drone/projected_map) goes over wifi to merge_map on the rover.

Use the matching rover.cyclone.octomap.launch.py on the rover so it does
not also try to run the drone octomap.

TF dependencies satisfied via DDS from the rover side:
  * rover/map  -> rover/odom        (rover LIO-SAM)
  * rover/map  -> ground            (static, from rover.cyclone.octomap)
  * rover/odom -> drone/odom        (camera_init_tf_from_raw_lidar)
Local to the drone:
  * drone/odom -> drone/lidar_link  (drone LIO-SAM)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── paths ─────────────────────────────────────────────────────────────
    livox_drone_launch_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'), 'launch', 'msg_MID360_drone_launch.py')

    #cyclonedds_uri = 'file://' + os.path.join(
    #    get_package_share_directory('mapping_launch'), 'config', 'cyclonedds.xml')

    # ── DDS configuration ─────────────────────────────────────────────────
    # Force CycloneDDS as RMW and point it at the workspace-shared XML so
    # discovery uses unicast peers (drone Pi <-> rover laptop) instead of
    # wifi multicast. Must be set before any node spawns.
    #set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    #set_cyclone_uri = SetEnvironmentVariable('CYCLONEDDS_URI', cyclonedds_uri)

    # ── arguments ─────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')

    # ══════════════════════════════════════════════════════════════════════
    # 1. HARDWARE
    # ══════════════════════════════════════════════════════════════════════

    # Livox MID360 driver for the drone
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_drone_launch_path),
    )

    # ══════════════════════════════════════════════════════════════════════
    # 2. CALIBRATION
    # ══════════════════════════════════════════════════════════════════════

    # Pointcloud aggregator starts 10 s after livox to let the LiDAR stabilise.
    # It waits for the rover's gather_aggregated_clouds to call the trigger
    # service before accumulating and then exits.
    aggregator_node = Node(
        package='calibrate_lidars',
        executable='pointcloud_aggregator_node',
        name='pointcloud_aggregator_node',
        namespace='drone',
        output='screen',
        parameters=[
            {'lidar_topic': 'livox/lidar'},
            {'aggregated_topic': 'calibration_pointcloud'},
            {'trigger_service': 'trigger_accumulation'},
            {'output_frame_id': 'drone_lidar'},
            {'messages_to_accumulate': 5},
            {'downsample_leaf_size': 0.05},
            {'min_dist': 1.0},
            {'max_dist': 10.0},
            {'use_sim_time': use_sim_time},
        ],
    )

    drone_lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('lio_sam'), 'launch', 'run.launch.py')),
        launch_arguments={
            'namespace': 'drone',
            'rviz': 'false',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ══════════════════════════════════════════════════════════════════════
    # 3. OCTOMAP (drone-side)
    # ══════════════════════════════════════════════════════════════════════
    # Runs on the drone Pi so the deskewed pointcloud (~50 KB @ 10 Hz) is
    # consumed locally instead of being shipped over wifi to the rover.
    # Only the resulting /drone/projected_map crosses the network to the
    # rover's merge_map_node.
    #
    # frame_id = rover/map so merge_map sees both projected maps in the
    # same frame. The cross-robot offset is absorbed at insertion via the
    # rover/odom -> drone/odom calibration TF, which is broadcast over DDS
    # by camera_init_tf_from_raw_lidar on the rover.
    drone_octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        namespace='drone',
        output='screen',
        parameters=[{
            'frame_id': 'rover/map',
            'resolution': 0.4,
            'base_frame_id': 'ground',
            'filter_speckles': True,
            'filter_ground_plane': True,
            'ground_filter.angle': 0.15,
            'ground_filter.distance': 0.3,
            'ground_filter.plane_distance': 0.2,
            'sensor_model.max_range': 8.0,
            # Bounds in drone/odom. Floor near z=-0.26; widen to absorb
            # slope drift in odom.
            'point_cloud_min_z': -1.0,
            'point_cloud_max_z': 1.5,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud_in', 'lio_sam/mapping/cloud_deskewed_sync'),
            ('/projected_map', 'map'),
            ('/octomap_binary', 'octomap_binary'),
            ('/octomap_full', 'octomap_full'),
            ('/octomap_point_cloud_centers', 'octomap_point_cloud_centers'),
            ('/occupied_cells_vis_array', 'occupied_cells_vis_array'),
            ('/free_cells_vis_array', 'free_cells_vis_array'),
        ],
    )


    return LaunchDescription([
        # DDS env vars MUST come before any Node/IncludeLaunchDescription so
        # every spawned process inherits CycloneDDS + the unicast peer config.
        #set_rmw,
        #set_cyclone_uri,

        declare_use_sim_time_cmd,

        # Nodes
        livox_driver,
        TimerAction(period=10.0, actions=[aggregator_node]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=aggregator_node,
                on_exit=[TimerAction(period=20.0, actions=[drone_lio_sam])],
            )
        ),
        # Octomap starts 35 s after aggregator exits — same total delay the
        # rover-side variant used, so calibration (rover/odom -> drone/odom)
        # has time to publish before octomap does its first TF lookup.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=aggregator_node,
                on_exit=[TimerAction(period=35.0, actions=[drone_octomap_node])],
            )
        ),
    ])
