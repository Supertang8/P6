import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np


class CostmapExpander(Node):
    def __init__(self):
        super().__init__('costmap_expander')

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.goal = None

        self.create_subscription(
            OccupancyGrid,
            '/merge_map',
            self.costmap_callback,
            10)

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos)

    def goal_callback(self, msg):
        self.goal = msg

    def costmap_callback(self, msg):
        self.costmap = msg
        if self.goal is None:
            self.pub.publish(msg)
            return

        cm = self.costmap
        res = cm.info.resolution

        origin_x = cm.info.origin.position.x
        origin_y = cm.info.origin.position.y

        width = cm.info.width
        height = cm.info.height

        max_x = origin_x + width * res
        max_y = origin_y + height * res

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        # Check if inside
        if origin_x <= gx <= max_x and origin_y <= gy <= max_y:
            self.pub.publish(msg)
            return  # no need to expand

        # Compute new bounds
        new_min_x = min(origin_x, gx) - 2.0
        new_min_y = min(origin_y, gy) - 2.0
        new_max_x = max(max_x, gx) + 2.0
        new_max_y = max(max_y, gy) + 2.0

        new_width = int(np.ceil((new_max_x - new_min_x) / res))
        new_height = int(np.ceil((new_max_y - new_min_y) / res))

        # Create new grid filled with unknown
        new_data = -np.ones((new_height, new_width), dtype=np.int8)

        # Convert old data into 2D
        old_data = np.array(cm.data, dtype=np.int8).reshape((height, width))

        # Compute offset
        offset_x = int(round((origin_x - new_min_x) / res))
        offset_y = int(round((origin_y - new_min_y) / res))

        # Copy old map into new
        new_data[
            offset_y:offset_y + height,
            offset_x:offset_x + width
        ] = old_data

        # Flatten
        new_msg = OccupancyGrid()
        new_msg.header = cm.header
        new_msg.header.stamp = self.get_clock().now().to_msg()

        new_msg.info.resolution = res
        new_msg.info.width = new_width
        new_msg.info.height = new_height
        new_msg.info.origin.position.x = new_min_x
        new_msg.info.origin.position.y = new_min_y
        new_msg.info.origin.orientation = cm.info.origin.orientation

        new_msg.data = new_data.flatten().tolist()

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapExpander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()