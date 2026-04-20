<<<<<<< Updated upstream
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2

# 👇 IMPORT FROM EXISTING PACKAGE
from livox_ros_driver2.msg import CustomMsg, CustomPoint


class PC2ToLivox(Node):

    def __init__(self):
        super().__init__('pc2_to_livox')
=======
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg, CustomPoint

import sensor_msgs_py.point_cloud2 as pc2


class PointCloudToLivox(Node):

    def __init__(self):
        super().__init__('pointcloud_to_livox')
>>>>>>> Stashed changes

        self.sub = self.create_subscription(
            PointCloud2,
            '/rover_lidar/points',
<<<<<<< Updated upstream
            self.lidar_callback,
=======
            self.callback,
>>>>>>> Stashed changes
            10
        )

        self.pub = self.create_publisher(
            CustomMsg,
<<<<<<< Updated upstream
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
=======
            '/rover_livox/lidar',
            10
        )

    def callback(self, msg):

        livox_msg = CustomMsg()

        livox_msg.header = msg.header
        livox_msg.timebase = self.get_clock().now().nanoseconds
        livox_msg.point_num = 0
        livox_msg.lidar_id = 0

        points = []

        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):

            pt = CustomPoint()

            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])

            pt.reflectivity = int(p[3])
            pt.tag = 0
            pt.line = 0

            pt.offset_time = 0

            points.append(pt)
>>>>>>> Stashed changes

        livox_msg.points = points
        livox_msg.point_num = len(points)

        self.pub.publish(livox_msg)


def main(args=None):
<<<<<<< Updated upstream
    rclpy.init(args=args)
    node = PC2ToLivox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
=======

    rclpy.init(args=args)

    node = PointCloudToLivox()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
>>>>>>> Stashed changes
