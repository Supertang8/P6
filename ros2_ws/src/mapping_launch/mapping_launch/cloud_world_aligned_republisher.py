"""Republish FAST-LIO's cloud_registered in a body-positioned, world-aligned frame.

Octomap with `filter_ground_plane: true` derives its sensor origin from the
cloud's frame_id position, and runs the cloud through a `body -> base_link ->
world` round-trip. Feeding it `cloud_registered_body` puts the body's full
attitude in that round-trip, which has shown up as occasional mirroring of the
occupied cells. Feeding it `cloud_registered` removes the mirroring but pins
the sensor origin to (0, 0, 0), capping `max_range` insertion to a fixed
sphere at the world origin.

This node sits between FAST-LIO and Octomap: it consumes `cloud_registered`
(world-frame points), subtracts the body's translation from each point, and
republishes them tagged with a new frame whose only relation to camera_init is
that translation. It also publishes the corresponding TF
`<parent>/camera_init -> <ns>/sensor_world_aligned` (identity rotation, body
translation) from the same Odometry message, in the same callback, so the
shift applied to the points and the translation in the TF are guaranteed to
match.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


_DTYPE_MAP = {
    PointField.INT8: 'i1',
    PointField.UINT8: 'u1',
    PointField.INT16: 'i2',
    PointField.UINT16: 'u2',
    PointField.INT32: 'i4',
    PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4',
    PointField.FLOAT64: 'f8',
}


def _cloud_struct_dtype(fields, point_step: int) -> np.dtype:
    names: list[str] = []
    formats: list[str] = []
    offsets: list[int] = []
    for f in fields:
        if f.count != 1 or f.datatype not in _DTYPE_MAP:
            continue
        names.append(f.name)
        formats.append(_DTYPE_MAP[f.datatype])
        offsets.append(f.offset)
    return np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': point_step,
    })


class CloudWorldAlignedRepublisher(Node):
    """Republish a world-frame cloud in a body-positioned, world-aligned frame."""

    def __init__(self) -> None:
        super().__init__('cloud_world_aligned_republisher')

        self.declare_parameter('odom_topic', 'Odometry')
        self.declare_parameter('cloud_topic', 'cloud_registered')
        self.declare_parameter('output_cloud_topic', 'cloud_registered_world_aligned')
        self.declare_parameter('output_frame', 'sensor_world_aligned')
        self.declare_parameter('parent_frame', '')
        self.declare_parameter('match_tolerance_ns', 50_000_000)
        self.declare_parameter('odom_buffer_size', 50)

        odom_topic = str(self.get_parameter('odom_topic').value)
        cloud_topic = str(self.get_parameter('cloud_topic').value)
        output_cloud_topic = str(self.get_parameter('output_cloud_topic').value)
        self._parent_frame_override = str(self.get_parameter('parent_frame').value)
        self._match_tol_ns = int(self.get_parameter('match_tolerance_ns').value)
        self._odom_buffer_max = int(self.get_parameter('odom_buffer_size').value)

        ns = self.get_namespace().strip('/')
        out_frame_param = str(self.get_parameter('output_frame').value).strip('/')
        if ns and '/' not in out_frame_param:
            self._output_frame = f'{ns}/{out_frame_param}'
        else:
            self._output_frame = out_frame_param

        self._tf = TransformBroadcaster(self)
        self._pub = self.create_publisher(PointCloud2, output_cloud_topic, 10)

        # Buffer of (stamp_ns, (x, y, z), parent_frame). Short-list, scanned linearly.
        self._odom_buffer: list[tuple[int, tuple[float, float, float], str]] = []

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, qos)

        self.get_logger().info(
            f'Republishing {cloud_topic} -> {output_cloud_topic} '
            f'in frame {self._output_frame}'
        )

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _odom_cb(self, msg: Odometry) -> None:
        stamp_ns = self._stamp_ns(msg.header.stamp)
        p = msg.pose.pose.position
        pos = (float(p.x), float(p.y), float(p.z))
        parent = self._parent_frame_override or msg.header.frame_id

        self._odom_buffer.append((stamp_ns, pos, parent))
        if len(self._odom_buffer) > self._odom_buffer_max:
            self._odom_buffer.pop(0)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = self._output_frame
        tf_msg.transform.translation.x = pos[0]
        tf_msg.transform.translation.y = pos[1]
        tf_msg.transform.translation.z = pos[2]
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self._tf.sendTransform(tf_msg)

    def _match_pose(self, cloud_stamp_ns: int) -> Optional[tuple[float, float, float]]:
        if not self._odom_buffer:
            return None
        best_pos: Optional[tuple[float, float, float]] = None
        best_dt: Optional[int] = None
        for stamp_ns, pos, _ in self._odom_buffer:
            dt = abs(stamp_ns - cloud_stamp_ns)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_pos = pos
                if dt == 0:
                    break
        if best_dt is not None and best_dt <= self._match_tol_ns:
            return best_pos
        return None

    def _cloud_cb(self, msg: PointCloud2) -> None:
        pos = self._match_pose(self._stamp_ns(msg.header.stamp))
        if pos is None:
            self.get_logger().warn(
                'No matching Odometry within tolerance for cloud; dropping.',
                throttle_duration_sec=5.0,
            )
            return

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._output_frame
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense

        n_points = msg.width * msg.height
        if n_points == 0:
            out.data = msg.data
            self._pub.publish(out)
            return

        if msg.is_bigendian:
            self.get_logger().error(
                'Big-endian PointCloud2 not supported by this republisher.',
                throttle_duration_sec=5.0,
            )
            return

        dt = _cloud_struct_dtype(msg.fields, msg.point_step)
        if not all(name in dt.names for name in ('x', 'y', 'z')):
            self.get_logger().error(
                'Input cloud is missing x/y/z fields; cannot shift.',
                throttle_duration_sec=5.0,
            )
            return

        # bytearray gives a writable buffer; np.frombuffer over it is mutable
        # and aliases the same memory we then publish.
        buf = bytearray(msg.data)
        arr = np.frombuffer(buf, dtype=dt, count=n_points)
        arr['x'] -= pos[0]
        arr['y'] -= pos[1]
        arr['z'] -= pos[2]

        out.data = bytes(buf)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CloudWorldAlignedRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
