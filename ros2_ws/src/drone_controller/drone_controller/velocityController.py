#!/usr/bin/env python3
"""Velocity controller that receives position setpoints and computes velocity commands using a P-controller."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class VelocityController(Node):
    def __init__(self):
        super().__init__("velocity_controller")

        self.desired_pose = None
        self.current_pos = None

        # P-controller gains
        self.kp_xy = 0.5  # Proportional gain for x and y
        self.kp_z = 0.5   # Proportional gain for z

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriber for desired position setpoints
        self.setpoint_sub = self.create_subscription(
            PoseStamped, "/offboard/setpoint", self.setpoint_callback, qos
        )

        # Subscriber for current position (from PX4)
        self.pose_sub = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.pose_callback, qos
        )

        # Publisher for velocity commands (ROS standard)
        self.twist_pub = self.create_publisher(TwistStamped, "/offboard/velocity", qos)

        # Timer to compute and publish velocity at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(3.0, self.status_loop)
        self.have_published = False

        self.get_logger().info("Velocity controller initialized.")
        self.get_logger().info("Waiting for /offboard/setpoint and /fmu/out/vehicle_local_position_v1 messages...")

    def setpoint_callback(self, msg: PoseStamped):
        self.desired_pose = msg
        self.get_logger().info(
            f"Received setpoint: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}"
        )

    def pose_callback(self, msg: VehicleLocalPosition):
        self.current_pos = [msg.x, msg.y, msg.z]
       

    def control_loop(self):
        if self.desired_pose is None or self.current_pos is None:
            return

        # Extract positions
        dx = self.desired_pose.pose.position.x - self.current_pos[0]
        dy = self.desired_pose.pose.position.y - self.current_pos[1]
        dz = self.desired_pose.pose.position.z - self.current_pos[2]

        # P-controller
        vx = self.kp_xy * dx
        vy = self.kp_xy * dy
        vz = self.kp_z * dz

        # Create velocity message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "map"
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.twist_pub.publish(twist_msg)
        

    def status_loop(self):
        if self.desired_pose is None and self.current_pos is None:
            self.get_logger().info("Still waiting for both setpoint and current position.")
        elif self.desired_pose is None:
            self.get_logger().info("Waiting for setpoint messages on /offboard/setpoint...")
        elif self.current_pos is None:
            self.get_logger().info("Waiting for current position messages on /fmu/out/vehicle_local_position_v1...")


def main() -> None:
    rclpy.init()
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
