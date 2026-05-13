import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from px4_msgs.msg import SensorCombined
from livox_ros_driver2.msg import CustomMsg


class PC2ToLivox(Node):

    def __init__(self):
        super().__init__('pc2_to_livox')

        # Initialize timing variables
        self.last_lidar_stamp = None
        self.last_lidar_arrival_time = None

        # Relative topic names
        self.imu_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.imu_callback,
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            'livox/imu',
            10
        )

        self.lidar_sub = self.create_subscription(
            CustomMsg,
            'livox/lidar',
            self.lidar_callback,
            10
        )

    def imu_callback(self, msg):
        if self.last_lidar_stamp is None:
            return  # No lidar yet, skip

        now = self.get_clock().now()

        # Time difference since last lidar
        dt = now - self.last_lidar_arrival_time

        # New timestamp = lidar stamp + dt
        new_stamp = self.last_lidar_stamp + dt
        imu_msg = Imu()

        imu_msg.header.stamp = new_stamp.to_msg()
        imu_msg.header.frame_id = "drone/base_link"

        # PX4 SensorCombined values
        # gyro_rad[0:3]
        imu_msg.angular_velocity.x = float(msg.gyro_rad[0])
        imu_msg.angular_velocity.y = float(msg.gyro_rad[1])
        imu_msg.angular_velocity.z = float(msg.gyro_rad[2])

        # accelerometer_m_s2[0:3]
        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
        imu_msg.linear_acceleration.z = float(msg.accelerometer_m_s2[2])

        self.imu_pub.publish(imu_msg)
        
    def lidar_callback(self, msg):
        self.last_lidar_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.last_lidar_arrival_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    node = PC2ToLivox()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()