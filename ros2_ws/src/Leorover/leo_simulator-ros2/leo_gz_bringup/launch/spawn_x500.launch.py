import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def spawn_drone(context: LaunchContext, namespace: LaunchConfiguration):

    pkg_description = get_package_share_directory("drone_description")

    drone_ns = context.perform_substitution(namespace)

    sdf_file = os.path.join(
        pkg_description,
        "sdf",
        "x500",
        "model.sdf",
    )

    if drone_ns == "":
        drone_name = "x500"
        prefix = ""
    else:
        drone_name = "x500" + drone_ns
        prefix = drone_ns + "_"

    spawn_entity = Node(
        namespace=drone_ns,
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            sdf_file,
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
           "/model/x500/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            "/world/default/model/x500/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen",
    )

    return [
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