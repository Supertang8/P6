import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2

# 👇 IMPORT FROM EXISTING PACKAGE
from livox_ros_driver2.msg import CustomMsg, CustomPoint


class PC2ToLivox(Node):

    def __init__(self):
        super().__init__('pc2_to_livox')

        self.sub = self.create_subscription(
            PointCloud2,
            '/rover_lidar/points',
            self.lidar_callback,
            10
        )

        self.pub = self.create_publisher(
            CustomMsg,
            '/livox/lidar',
            10
        )

    def lidar_callback(self, msg):
        livox_msg = CustomMsg()

        # Header
        livox_msg.header = msg.header
        livox_msg.timebase = (
            msg.header.stamp.sec * 1_000_000_000 +
            msg.header.stamp.nanosec
        )

        livox_msg.lidar_id = 0
        livox_msg.rsvd = [0, 0, 0]

        points = []

        for i, p in enumerate(point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True)):

            cp = CustomPoint()

            x, y, z, intensity = p

            cp.x = float(x)
            cp.y = float(y)
            cp.z = float(z)
            cp.reflectivity = int(intensity) if intensity else 0

            # ⚠️ Replace if your cloud has "time"
            cp.offset_time = i

            cp.tag = 0
            cp.line = 0

            points.append(cp)

        livox_msg.points = points
        livox_msg.point_num = len(points)

        self.pub.publish(livox_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PC2ToLivox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
