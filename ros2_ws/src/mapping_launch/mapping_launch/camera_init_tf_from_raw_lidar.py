"""Compute inter-robot camera_init transform from a raw LiDAR-LiDAR transform.

The node waits for one odometry message from each robot, converts the provided
raw LiDAR transform into camera_init coordinates, and publishes one static TF:
parent camera_init -> child camera_init.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _normalize_quaternion(q: list[float]) -> list[float]:
    x, y, z, w = q
    n = sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x / n, y / n, z / n, w / n]


def _quat_conjugate(q: list[float]) -> list[float]:
    x, y, z, w = q
    return [-x, -y, -z, w]


def _quat_multiply(qa: list[float], qb: list[float]) -> list[float]:
    ax, ay, az, aw = qa
    bx, by, bz, bw = qb
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def _rotate_vector(q: list[float], v: list[float]) -> list[float]:
    qv = [v[0], v[1], v[2], 0.0]
    q_conj = _quat_conjugate(q)
    q_tmp = _quat_multiply(q, qv)
    q_out = _quat_multiply(q_tmp, q_conj)
    return [q_out[0], q_out[1], q_out[2]]


@dataclass
class Transform:
    t: list[float]  # [x, y, z]
    q: list[float]  # [x, y, z, w]


def _compose(a_b: Transform, b_c: Transform) -> Transform:
    q_ab = _normalize_quaternion(a_b.q)
    q_bc = _normalize_quaternion(b_c.q)
    q_ac = _normalize_quaternion(_quat_multiply(q_ab, q_bc))
    t_rot = _rotate_vector(q_ab, b_c.t)
    t_ac = [
        a_b.t[0] + t_rot[0],
        a_b.t[1] + t_rot[1],
        a_b.t[2] + t_rot[2],
    ]
    return Transform(t=t_ac, q=q_ac)


def _inverse(a_b: Transform) -> Transform:
    q_ab = _normalize_quaternion(a_b.q)
    q_ba = _quat_conjugate(q_ab)
    t_ba = _rotate_vector(q_ba, [-a_b.t[0], -a_b.t[1], -a_b.t[2]])
    return Transform(t=t_ba, q=q_ba)


def _topic_with_namespace(namespace: str, topic: str) -> str:
    ns = namespace.strip('/')
    top = topic.lstrip('/')
    if not ns:
        return f'/{top}'
    return f'/{ns}/{top}'


def _parse_vec_param(value, expected_len: int, param_name: str) -> list[float]:
    if isinstance(value, str):
        parts = [v.strip() for v in value.split(',') if v.strip()]
        if len(parts) != expected_len:
            raise ValueError(
                f'Parameter {param_name} expects {expected_len} values, got {len(parts)}: {value}'
            )
        return [float(v) for v in parts]
    values = [float(v) for v in value]
    if len(values) != expected_len:
        raise ValueError(
            f'Parameter {param_name} expects {expected_len} values, got {len(values)}: {values}'
        )
    return values


class CameraInitTfFromRawLidar(Node):
    """Publish corrected camera_init-parent -> camera_init-child static TF."""

    def __init__(self) -> None:
        super().__init__('camera_init_tf_from_raw_lidar')

        self.declare_parameter('parent_namespace', 'rover')
        self.declare_parameter('child_namespace', 'drone')
        self.declare_parameter('lookup_rate_hz', 5.0)

        self.declare_parameter('raw_lidar_parent_to_child_xyz', '0.0,0.0,0.0')
        self.declare_parameter('raw_lidar_parent_to_child_xyzw', '0.0,0.0,0.0,1.0')

        # T_B_L for each robot (body/IMU -> LiDAR), from FAST_LIO extrinsics.
        self.declare_parameter('parent_body_to_lidar_xyz', '0.0,0.0,0.0')
        self.declare_parameter('parent_body_to_lidar_xyzw', '0.0,0.0,0.0,1.0')
        self.declare_parameter('child_body_to_lidar_xyz', '0.0,0.0,0.0')
        self.declare_parameter('child_body_to_lidar_xyzw', '0.0,0.0,0.0,1.0')

        self.parent_namespace = str(self.get_parameter('parent_namespace').value)
        self.child_namespace = str(self.get_parameter('child_namespace').value)
        self.lookup_rate_hz = float(self.get_parameter('lookup_rate_hz').value)

        self.t_lidar_parent_child = Transform(
            t=_parse_vec_param(
                self.get_parameter('raw_lidar_parent_to_child_xyz').value,
                3,
                'raw_lidar_parent_to_child_xyz',
            ),
            q=_parse_vec_param(
                self.get_parameter('raw_lidar_parent_to_child_xyzw').value,
                4,
                'raw_lidar_parent_to_child_xyzw',
            ),
        )
        self.t_parent_body_lidar = Transform(
            t=_parse_vec_param(
                self.get_parameter('parent_body_to_lidar_xyz').value,
                3,
                'parent_body_to_lidar_xyz',
            ),
            q=_parse_vec_param(
                self.get_parameter('parent_body_to_lidar_xyzw').value,
                4,
                'parent_body_to_lidar_xyzw',
            ),
        )
        self.t_child_body_lidar = Transform(
            t=_parse_vec_param(
                self.get_parameter('child_body_to_lidar_xyz').value,
                3,
                'child_body_to_lidar_xyz',
            ),
            q=_parse_vec_param(
                self.get_parameter('child_body_to_lidar_xyzw').value,
                4,
                'child_body_to_lidar_xyzw',
            ),
        )

        self._parent_world_to_body: Optional[Transform] = None
        self._child_world_to_body: Optional[Transform] = None
        self._parent_world_frame: Optional[str] = None
        self._child_world_frame: Optional[str] = None
        self._published = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        timer_period = 1.0 / self.lookup_rate_hz if self.lookup_rate_hz > 0.0 else 0.2
        self._timer = self.create_timer(timer_period, self._try_publish)

        self.get_logger().info(
            'Waiting for FAST_LIO TFs: '
            f'parent={self.parent_namespace}/camera_init -> {self.parent_namespace}/body, '
            f'child={self.child_namespace}/camera_init -> {self.child_namespace}/body'
        )

        self._broadcaster = StaticTransformBroadcaster(self)

    @staticmethod
    def _transform_stamped_to_transform(msg: TransformStamped) -> Transform:
        p = msg.transform.translation
        o = msg.transform.rotation
        return Transform(t=[p.x, p.y, p.z], q=[o.x, o.y, o.z, o.w])

    def _try_publish(self) -> None:
        if self._published:
            return

        parent_target = f'{self.parent_namespace}/camera_init'
        parent_source = f'{self.parent_namespace}/body'
        child_target = f'{self.child_namespace}/camera_init'
        child_source = f'{self.child_namespace}/body'

        try:
            parent_tf = self._tf_buffer.lookup_transform(parent_target, parent_source, rclpy.time.Time())
            child_tf = self._tf_buffer.lookup_transform(child_target, child_source, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        self._parent_world_to_body = self._transform_stamped_to_transform(parent_tf)
        self._child_world_to_body = self._transform_stamped_to_transform(child_tf)
        self._parent_world_frame = parent_tf.header.frame_id
        self._child_world_frame = child_tf.header.frame_id

        # T_Wp_Lp = T_Wp_Bp * T_Bp_Lp
        t_wp_lp = _compose(self._parent_world_to_body, self.t_parent_body_lidar)
        # T_Wc_Lc = T_Wc_Bc * T_Bc_Lc
        t_wc_lc = _compose(self._child_world_to_body, self.t_child_body_lidar)
        # T_Wp_Wc = T_Wp_Lp * T_Lp_Lc * inv(T_Wc_Lc)
        t_wp_wc = _compose(_compose(t_wp_lp, self.t_lidar_parent_child), _inverse(t_wc_lc))

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self._parent_world_frame
        tf_msg.child_frame_id = self._child_world_frame
        tf_msg.transform.translation.x = t_wp_wc.t[0]
        tf_msg.transform.translation.y = t_wp_wc.t[1]
        tf_msg.transform.translation.z = t_wp_wc.t[2]
        tf_msg.transform.rotation.x = t_wp_wc.q[0]
        tf_msg.transform.rotation.y = t_wp_wc.q[1]
        tf_msg.transform.rotation.z = t_wp_wc.q[2]
        tf_msg.transform.rotation.w = t_wp_wc.q[3]

        self._broadcaster.sendTransform(tf_msg)
        self._published = True

        self.get_logger().info(
            'Published static TF '
            f'{tf_msg.header.frame_id} -> {tf_msg.child_frame_id} from raw LiDAR transform.'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraInitTfFromRawLidar()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()