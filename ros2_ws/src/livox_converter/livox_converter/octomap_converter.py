import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PC2ToLivox(Node):

    def __init__(self):
        super().__init__('pc2_to_livox')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PC2ToLivox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()