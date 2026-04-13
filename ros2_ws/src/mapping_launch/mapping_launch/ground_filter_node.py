import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def quaternion_to_matrix(x, y, z, w):
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float32)


class GroundFilterNode(Node):
    def __init__(self):
        super().__init__('ground_filter_node')

        self.declare_parameter('input_cloud_topic', '/cloud_registered')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('output_topic', '/cloud_ground_filtered_body')
        self.declare_parameter('output_frame_id', 'body')
        self.declare_parameter('ground_roi_radius', 6.0)
        self.declare_parameter('ground_percentile', 12.0)
        self.declare_parameter('ground_clearance', 0.22)
        self.declare_parameter('max_above_ground', 1.0)
        self.declare_parameter('min_range', 0.8)
        self.declare_parameter('max_range', 20.0)

        self._position = None
        self._rotation_world_from_body = None

        self._output_frame_id = self.get_parameter('output_frame_id').value
        self._ground_roi_radius = float(self.get_parameter('ground_roi_radius').value)
        self._ground_percentile = float(self.get_parameter('ground_percentile').value)
        self._ground_clearance = float(self.get_parameter('ground_clearance').value)
        self._max_above_ground = float(self.get_parameter('max_above_ground').value)
        self._min_range = float(self.get_parameter('min_range').value)
        self._max_range = float(self.get_parameter('max_range').value)

        input_cloud_topic = self.get_parameter('input_cloud_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._cloud_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._cloud_sub = self.create_subscription(PointCloud2, input_cloud_topic, self._on_cloud, 10)

        self.get_logger().info(
            f'Ground filter active: {input_cloud_topic} -> {output_topic}, frame={self._output_frame_id}')

    def _on_odom(self, msg):
        pose = msg.pose.pose
        self._position = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ], dtype=np.float32)
        self._rotation_world_from_body = quaternion_to_matrix(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

    def _on_cloud(self, msg):
        if self._position is None or self._rotation_world_from_body is None:
            return

        field_names = [field.name for field in msg.fields]
        has_intensity = 'intensity' in field_names
        read_fields = ['x', 'y', 'z'] + (['intensity'] if has_intensity else [])

        raw_points = point_cloud2.read_points(msg, field_names=read_fields, skip_nans=True)

        # ROS2 Humble may return either a generator of tuples or a structured numpy array.
        if isinstance(raw_points, np.ndarray) and raw_points.dtype.names is not None:
            if has_intensity:
                points = np.column_stack((
                    raw_points['x'],
                    raw_points['y'],
                    raw_points['z'],
                    raw_points['intensity'],
                )).astype(np.float32)
            else:
                points = np.column_stack((
                    raw_points['x'],
                    raw_points['y'],
                    raw_points['z'],
                )).astype(np.float32)
        else:
            points = np.asarray(list(raw_points), dtype=np.float32)

        if points.size == 0:
            return

        xyz_world = points[:, :3]
        rel_xy = xyz_world[:, :2] - self._position[:2]
        xy_range = np.hypot(rel_xy[:, 0], rel_xy[:, 1])

        range_mask = (xy_range >= self._min_range) & (xy_range <= self._max_range)
        roi_mask = range_mask & (xy_range <= self._ground_roi_radius)

        candidate_points = xyz_world[roi_mask]
        if candidate_points.shape[0] < 200:
            candidate_points = xyz_world[range_mask]
        if candidate_points.shape[0] < 200:
            candidate_points = xyz_world

        ground_z = float(np.percentile(candidate_points[:, 2], self._ground_percentile))
        keep_mask = (
            range_mask
            & (xyz_world[:, 2] > ground_z + self._ground_clearance)
            & (xyz_world[:, 2] < ground_z + self._max_above_ground)
        )

        kept_world = xyz_world[keep_mask]
        if kept_world.shape[0] == 0:
            return

        rotation_body_from_world = self._rotation_world_from_body.T
        kept_body = (rotation_body_from_world @ (kept_world - self._position).T).T.astype(np.float32)

        if has_intensity:
            kept_intensity = points[keep_mask, 3:4]
            output_points = np.hstack((kept_body, kept_intensity))
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
        else:
            output_points = kept_body
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

        header = msg.header
        header.frame_id = self._output_frame_id
        filtered_msg = point_cloud2.create_cloud(header, fields, output_points.tolist())
        self._cloud_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()