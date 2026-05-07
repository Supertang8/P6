import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from px4_msgs.msg import VehicleLocalPosition

import tf2_ros
from tf2_ros import Buffer, TransformListener

class SafetyDome(Node):
    def __init__(self):
        super().__init__('safety_dome')

        self.marker_pub = self.create_publisher(Marker, '/safe_zone_marker', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone_pose', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pose_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.pose_callback,
            qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.update_marker)

        self.rover_frame = "body"   
        self.fixed_frame = "camera_init"
        self.rover_pos = [0.0, 0.0, 0.0]

    def update_marker(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.rover_frame,
                rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z
        except Exception:
            # Fallback to world origin 0,0,0 if TF is not available
            x, y, z = 0.0, 0.0, 0.0

        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "safety_dome"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.w = 1.0

        # 16 meter radius → diameter = 32
        marker.scale.x = 32.0
        marker.scale.y = 32.0
        marker.scale.z = 32.0

        # semi-transparent red dome
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.2

        self.marker_pub.publish(marker)

    def pose_callback(self, msg: VehicleLocalPosition):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'camera_init'

        pose.pose.position.x = float(msg.x)
        pose.pose.position.y = float(msg.y)
        pose.pose.position.z = float(-msg.z)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = SafetyDome()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()