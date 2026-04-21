#!/usr/bin/env python3
"""Send 4 position setpoints to offboard_control_pos with 5-second intervals.

Setpoints (NED frame):
  1. [  0,  0, -5 ] yaw=0     — hover at origin, 5 m altitude, facing north
  2. [ 10,  0, -5 ] yaw=0     — 10 m north
  3. [ 10, 10, -5 ] yaw=1.57  — 10 m north, 10 m east, facing east
  4. [  0,  0, -5 ] yaw=3.14  — back to origin, facing south
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


SETPOINTS = [
    # (north, east, down,  yaw_deg)
    (  0.0,  0.0, -5.0,  0.0),
    (  15.0, 0.0, -5.0,  0.0),
    (  0.0, 0.0, -5.0,  0.0),
    (  15.0, 0.0, -5.0,  0.0),
    (  0.0,  0.0, -5.0,  0.0),

    #Z hover test
    # (0.0, 0.0, -3.0, 0.0),
    # (0.0, 0.0, -6.0, 0.0),
    # (0.0, 0.0, -3.0, 0.0),
    # (0.0, 0.0, -6.0, 0.0),
    # (0.0, 0.0, -3.0, 0.0),
    # (0.0, 0.0, -6.0, 0.0),
    # (0.0, 0.0, -3.0, 0.0),
    # (0.0, 0.0, -6.0, 0.0),
]

INTERVAL_S = 15.0


def make_msg(north: float, east: float, down: float, yaw_deg: float) -> PoseStamped:
    yaw = math.radians(yaw_deg)
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = north
    msg.pose.position.y = east
    msg.pose.position.z = down
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    return msg


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("setpoint_sender")

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    pub = node.create_publisher(PoseStamped, "/offboard/setpoint", qos)

    # Give the publisher time to connect before sending the first message
    time.sleep(1.0)

    for i, (north, east, down, yaw_deg) in enumerate(SETPOINTS):
        msg = make_msg(north, east, down, yaw_deg)
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        node.get_logger().info(
            f"[{i + 1}/{len(SETPOINTS)}] NED [{north}, {east}, {down}]  yaw {yaw_deg}°"
        )
        if i < len(SETPOINTS) - 1:
            time.sleep(INTERVAL_S)

    node.get_logger().info("All setpoints sent.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
