import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def spawn_drone(context: LaunchContext, namespace: LaunchConfiguration):

    pkg_description = get_package_share_directory("drone_description")

    drone_ns = context.perform_substitution(namespace)

    robot_desc = xacro.process(
        os.path.join(
            pkg_description, 
            "urdf", 
            "simple_drone.urdf.xacro"
        ),
        mappings={"drone_ns": drone_ns},
    )

    if drone_ns == "":
        drone_name = "simple_drone"
        prefix = ""
    else:
        drone_name = "simple_drone_" + drone_ns
        prefix = drone_ns + "_"

    robot_state_publisher = Node(
        namespace=drone_ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
        output="screen",
    )

    spawn_entity = Node(
        namespace=drone_ns,
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            drone_name,
            "-z",
            "1.0",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=prefix + "bridge",
        arguments=[
            drone_ns + "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            drone_ns + "/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
        ],
        output="screen",
    )

    return [
        robot_state_publisher,
        spawn_entity,
        bridge,
    ]


def generate_launch_description():

    ns_arg = DeclareLaunchArgument(
        "drone_ns",
        default_value="",
        description="Drone namespace",
    )

    namespace = LaunchConfiguration("drone_ns")

    return LaunchDescription(
        [ns_arg, OpaqueFunction(function=spawn_drone, args=[namespace])]
    )