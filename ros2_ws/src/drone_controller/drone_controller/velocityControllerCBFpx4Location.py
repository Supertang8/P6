#!/usr/bin/env python3
"""Velocity controller that receives position setpoints and computes velocity commands using a P-controller."""

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import TransformException


class VelocityController(Node):
    def __init__(self):
        super().__init__("velocity_controller")

        self.desired_pose = None
        self.current_pos = None

        # P-controller gains
        self.kp_xy = 0.9  # Proportional gain for x and y
        self.kp_z = 2.5  # Proportional gain for z

        # I-controller gains
        self.ki_xy = 0#0.01  # Integral gain for x and y
        self.ki_z = 0#0.02  # Integral gain for z
        # I-controller error accumulators
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_z = 0.0

        # D-controller gains
        self.kd_xy = 0.95#1.3  # Derivative gain for x and y
        self.kd_z = 1#0.95#1.3  # Derivative gain for z
        # D-controller error accumulators
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0

        # CBF parameters
        self.max_dist = 10.0 #Maximum distance from drone to rover
        self.alpha = 1.5 #CBF gain (TUNE, ofte mellem 1-3)
        ####### Rover state FOR TESTING PURPOSES, REPLACE WITH REAL SUBSCRIBER##########
        self.rover_pos = [0.0, 0.0, 0.0]  # [x,y,z]
        self.rover_vel = [0.0, 0.0, 0.0]

        # --- TF2 setup ---
        # Change these frame names to match your TF tree
        self.parent_frame_rover = "rover/camera_init"  # e.g. "world", "odom"
        self.child_frame_rover = "rover/body"  # e.g. "rover", "base_link"
        self.tf_buffer = tf2_ros.Buffer() 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        # Subscriber for rover velocity
        self.rover_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.rover_vel_callback, qos
        )

        # Publisher for velocity commands (ROS standard)
        self.twist_pub = self.create_publisher(TwistStamped, "/offboard/velocity", qos)

        # Timer to compute and publish velocity at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(3.0, self.status_loop)

        self.get_logger().info("Velocity controller initialized.")

    def setpoint_callback(self, msg: PoseStamped):
        self.desired_pose = msg
        self.get_logger().info(
            f"Received setpoint: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}"
        )
    
    def rover_vel_callback(self, msg: Twist):
        self.rover_vel = [msg.linear.x, msg.linear.y, msg.linear.z]

    def pose_callback(self, msg: VehicleLocalPosition):
        self.current_pos = [msg.x, msg.y, msg.z]
        
    def _update_rover_position_from_tf(self) -> bool:
        """Look up the latest rover transform and update self.rover_pos.
        Returns True on success, False if the transform is not yet available."""
        try:
            # rclpy.time.Time() means "latest available transform"
            ########### TILFØJ ROVER/
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame_rover,
                self.child_frame_rover,
                rclpy.time.Time(),
            )
            t = tf.transform.translation
            self.rover_pos = [t.x, t.y, t.z]
            self.get_logger().info(f"Current rover position from TF: x={t.x:.2f}, y={t.y:.2f}, z={t.z:.2f}")
            return True
        except TransformException as e:
            self.get_logger().warn(f"Could not get TF rover transform: {e}", throttle_duration_sec=2.0)
            return False

  
    def apply_cbf(self, ux, uy, uz):
        dx = self.current_pos[0] - self.rover_pos[0]
        dy = self.current_pos[1] - self.rover_pos[1]
        dz = self.current_pos[2] - self.rover_pos[2]

        h = self.max_dist**2 - (dx**2 + dy**2 + dz**2)
        u_rel = [ux - self.rover_vel[0], 
                    uy - self.rover_vel[1], 
                    uz - self.rover_vel[2]]
        
        h_dot = -2 * (dx * u_rel[0]+ dy * u_rel[1] + dz * u_rel[2])

        ### First we check if the current control input satisfies the CBF constraint
        if h_dot + self.alpha * h >= 0:
            return ux, uy, uz  # No modification needed
        ### If not, we project the control input onto the boundary of the safe set
        h_grad = [-2*dx, -2*dy, -2*dz]
        h_grad_norm = h_grad[0]**2 + h_grad[1]**2 + h_grad[2]**2

        ### If we're exactly at the boundary, stop (And avoid division by zero)
        if h_grad_norm < 1e-6:
            return 0.0, 0.0, 0.0  

        ### Compute the safe control input
        lambda_val = (-(self.alpha * h) - h_dot) / h_grad_norm
        ux_safe = ux + lambda_val * h_grad[0]
        uy_safe = uy + lambda_val * h_grad[1]
        uz_safe = uz + lambda_val * h_grad[2]

        return ux_safe, uy_safe, uz_safe
       

    def control_loop(self):
        # Refresh position from TF every tick
        self._update_rover_position_from_tf()
            

        if self.desired_pose is None or self.current_pos is None:
            return

        # Extract position differences
        dx = self.desired_pose.pose.position.x - self.current_pos[0]
        dy = self.desired_pose.pose.position.y - self.current_pos[1]
        dz = self.desired_pose.pose.position.z - self.current_pos[2]

        #print dx, dy, dz
        #self.get_logger().info(f"Position error: dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}")

        # P-gains
        px = self.kp_xy * dx
        py = self.kp_xy * dy
        pz = self.kp_z * dz
        
        # I-gains
        self.integral_error_x += dx * self.ki_xy
        self.integral_error_y += dy * self.ki_xy
        self.integral_error_z += dz * self.ki_z
        # Anti-windup: limit the integral error to prevent excessive accumulation
        max_integral = 1.0  # Maximum integral term
        ix = max(min(self.integral_error_x, max_integral), -max_integral)
        iy = max(min(self.integral_error_y, max_integral), -max_integral)
        iz = max(min(self.integral_error_z, max_integral), -max_integral)

        # D-error
        d_error_x = dx - self.prev_error_x
        d_error_y = dy - self.prev_error_y
        d_error_z = dz - self.prev_error_z
        self.prev_error_x = dx
        self.prev_error_y = dy
        self.prev_error_z = dz
        dx = self.kd_xy * d_error_x
        dy = self.kd_xy * d_error_y
        dz = self.kd_z * d_error_z

        #Control signal 
        ux = px + ix + dx
        uy = py + iy + dy
        uz = pz + iz + dz

        # Apply CBF
        ux, uy, uz = self.apply_cbf(ux, uy, uz)

        # Velocity limits
        max_vel_xy = 0.5  # m/s
        max_vel_z = 0.5   # m/s
        ux = max(min(ux, max_vel_xy), -max_vel_xy)
        uy = max(min(uy, max_vel_xy), -max_vel_xy)
        uz = max(min(uz, max_vel_z), -max_vel_z)
      

        # Create velocity message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "map"
        twist_msg.twist.linear.x = ux
        twist_msg.twist.linear.y = uy
        twist_msg.twist.linear.z = uz
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
            self.get_logger().info("Waiting for current position messages on /tf...")


def main() -> None:
    rclpy.init()
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
