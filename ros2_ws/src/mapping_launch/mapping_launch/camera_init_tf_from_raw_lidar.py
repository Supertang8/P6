"""Compute inter-robot map-frame transform from a raw LiDAR-LiDAR transform.

LIO-SAM publishes ``<ns>/odom -> <ns>/base_link`` (from imuPreintegration) and
the URDF supplies the static identity ``<ns>/base_link -> <ns>/livox_frame``,
so a TF lookup ``<ns>/odom -> <ns>/livox_frame`` returns the *gravity-aligned*
body pose. Multi_LiCa however calibrates the *raw* LiDAR frames (the
sensor's physical orientation, before LIO-SAM's internal extRot rotates
points into the gravity-aligned frame). To compose the two, this node
runs its own startup gravity calibration on each robot's IMU — mirroring
LIO-SAM's algorithm — to recover the per-robot extRot, then applies:

    T_(parent/map -> child/map) =
        T_lookup_p · extRot_p · T_LpLc · extRot_c^T · inv(T_lookup_c)

The result is published as ``<parent_ns>/map -> <child_ns>/map`` (rather
than odom -> odom) to avoid a parent collision with the per-robot
``<ns>/map -> <ns>/odom`` static published from LIO-SAM's run.launch.py.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from math import cos, radians, sin, sqrt
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


Vec3 = list[float]
Mat3 = list[list[float]]


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


_IDENTITY = Transform(t=[0.0, 0.0, 0.0], q=[0.0, 0.0, 0.0, 1.0])


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


def _quat_from_rpy_degrees(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list[float]:
    r, p, y = radians(roll_deg), radians(pitch_deg), radians(yaw_deg)
    cr, sr = cos(r * 0.5), sin(r * 0.5)
    cp, sp = cos(p * 0.5), sin(p * 0.5)
    cy, sy = cos(y * 0.5), sin(y * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def _quat_from_matrix(R: Mat3) -> list[float]:
    """Convert 3x3 rotation matrix to quaternion [x, y, z, w] (Shepperd's method)."""
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0.0:
        s = 0.5 / sqrt(trace + 1.0)
        return [
            (R[2][1] - R[1][2]) * s,
            (R[0][2] - R[2][0]) * s,
            (R[1][0] - R[0][1]) * s,
            0.25 / s,
        ]
    if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = 2.0 * sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
        return [0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s,
                (R[2][1] - R[1][2]) / s]
    if R[1][1] > R[2][2]:
        s = 2.0 * sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
        return [(R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s,
                (R[0][2] - R[2][0]) / s]
    s = 2.0 * sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
    return [(R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s,
            (R[1][0] - R[0][1]) / s]


def _gravity_rotation(samples_g: list[Vec3]) -> tuple[Optional[Mat3], float]:
    """Mirror LIO-SAM's gravity calibration (utility.hpp::collectGravitySample).

    Returns (R, mag) where R is the Rodrigues rotation that maps the measured
    gravity direction onto body +Z, or None if the |a|≈1g sanity check fails.
    Samples are expected in g (Livox MID360 convention).
    """
    n = len(samples_g)
    g = [sum(s[i] for s in samples_g) / n for i in range(3)]
    mag = sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2])
    if mag < 0.85 or mag > 1.15:
        return None, mag
    gn = [g[i] / mag for i in range(3)]
    # axis = gn × ẑ, cosA = gn · ẑ
    axis = [gn[1] * 1.0 - gn[2] * 0.0,
            gn[2] * 0.0 - gn[0] * 1.0,
            gn[0] * 0.0 - gn[1] * 0.0]
    axis_norm = sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    cos_a = max(-1.0, min(1.0, gn[2]))
    if axis_norm < 1e-9:
        if cos_a > 0.0:
            R: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        else:
            R = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
        return R, mag
    ku = [axis[0] / axis_norm, axis[1] / axis_norm, axis[2] / axis_norm]
    K: Mat3 = [
        [0.0, -ku[2], ku[1]],
        [ku[2], 0.0, -ku[0]],
        [-ku[1], ku[0], 0.0],
    ]
    KK: Mat3 = [
        [sum(K[i][k] * K[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]
    sin_a = axis_norm
    one_minus_cos = 1.0 - cos_a
    R = [
        [(1.0 if i == j else 0.0) + sin_a * K[i][j] + one_minus_cos * KK[i][j]
         for j in range(3)]
        for i in range(3)
    ]
    return R, mag


def _load_calibration_transform(path: str) -> Transform:
    with open(path, 'r') as f:
        text = f.read()
    num = r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?'
    xyz_match = re.search(rf'calibrated xyz\s*=\s*({num})\s+({num})\s+({num})', text)
    rpy_match = re.search(rf'calibrated rpy\s*=\s*({num})\s+({num})\s+({num})', text)
    if xyz_match is None or rpy_match is None:
        raise ValueError(f'Could not parse calibrated xyz/rpy from {path}')
    xyz = [float(g) for g in xyz_match.groups()]
    rpy_deg = [float(g) for g in rpy_match.groups()]
    return Transform(t=xyz, q=_quat_from_rpy_degrees(*rpy_deg))


@dataclass
class _RobotState:
    namespace: str
    samples: list[Vec3] = field(default_factory=list)
    ext_rot: Optional[Transform] = None  # gravity-aligning rotation as Transform
    sub: Optional[object] = None


class CameraInitTfFromRawLidar(Node):
    """Publish parent-map -> child-map static TF from Multi_LiCa + per-robot gravity calibration."""

    def __init__(self) -> None:
        super().__init__('camera_init_tf_from_raw_lidar')

        self.declare_parameter('parent_namespace', 'rover')
        self.declare_parameter('child_namespace', 'drone')
        self.declare_parameter('lookup_rate_hz', 5.0)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('lidar_frame', 'livox_frame')
        # Frame to publish on the static TF (defaults to 'map' so we don't
        # collide with run.launch.py's <ns>/map -> <ns>/odom static).
        self.declare_parameter('publish_frame', 'map')
        # Gravity calibration: matches LIO-SAM defaults (200 samples, ~1 s @
        # 200 Hz). Robot must be stationary during this window.
        self.declare_parameter('imu_topic', 'livox/imu')
        self.declare_parameter('imu_calibration_samples', 200)

        self.parent_namespace = str(self.get_parameter('parent_namespace').value)
        self.child_namespace = str(self.get_parameter('child_namespace').value)
        self.lookup_rate_hz = float(self.get_parameter('lookup_rate_hz').value)
        self.world_frame = str(self.get_parameter('world_frame').value).strip('/')
        self.lidar_frame = str(self.get_parameter('lidar_frame').value).strip('/')
        self.publish_frame = str(self.get_parameter('publish_frame').value).strip('/')
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.imu_samples_target = int(self.get_parameter('imu_calibration_samples').value)

        calibration_file = str(self.get_parameter('calibration_file').value)
        if not calibration_file:
            raise ValueError('Parameter calibration_file must be set to a valid file path')
        self.t_lidar_parent_child = _load_calibration_transform(calibration_file)

        self._published = False
        self._parent = _RobotState(namespace=self.parent_namespace)
        self._child = _RobotState(namespace=self.child_namespace)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        # IMU subscriptions (BEST_EFFORT, depth=2000 to match LIO-SAM utility.hpp).
        imu_qos = QoSProfile(
            depth=2000,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        for state in (self._parent, self._child):
            topic = _topic_with_namespace(state.namespace, self.imu_topic)
            state.sub = self.create_subscription(
                Imu, topic, self._make_imu_cb(state), imu_qos
            )

        timer_period = 1.0 / self.lookup_rate_hz if self.lookup_rate_hz > 0.0 else 0.2
        self._timer = self.create_timer(timer_period, self._try_publish)

        self.get_logger().info(
            'Waiting for IMU calibration ({} samples each from {} and {}) and TFs '
            '{}/{}/{} -> {}/{}/{} from each robot.'.format(
                self.imu_samples_target,
                _topic_with_namespace(self.parent_namespace, self.imu_topic),
                _topic_with_namespace(self.child_namespace, self.imu_topic),
                self.parent_namespace, self.world_frame, self.lidar_frame,
                self.child_namespace, self.world_frame, self.lidar_frame,
            )
        )

        self._broadcaster = StaticTransformBroadcaster(self)

        _transient_reliable = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._shutdown_pubs = [
            self.create_publisher(
                Empty,
                _topic_with_namespace(ns, 'shutdown'),
                _transient_reliable,
            )
            for ns in (self.parent_namespace, self.child_namespace)
        ]

    def _make_imu_cb(self, state: _RobotState):
        def _cb(msg: Imu) -> None:
            if state.ext_rot is not None:
                return
            state.samples.append([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ])
            if len(state.samples) >= self.imu_samples_target:
                R, mag = _gravity_rotation(state.samples)
                if R is None:
                    self.get_logger().warn(
                        f'[gravity-cal {state.namespace}] mean |a|={mag:.4f} g over '
                        f'{len(state.samples)} samples (expected ~1.0). Falling back '
                        f'to identity extRot — was the robot moving?'
                    )
                    state.ext_rot = _IDENTITY
                else:
                    state.ext_rot = Transform(t=[0.0, 0.0, 0.0], q=_quat_from_matrix(R))
                    self.get_logger().info(
                        f'[gravity-cal {state.namespace}] done after {len(state.samples)} '
                        f'samples. |a|={mag:.4f} g. extRot computed.'
                    )
                state.samples.clear()
                # Drop the subscription; we don't need any more samples.
                if state.sub is not None:
                    self.destroy_subscription(state.sub)
                    state.sub = None
        return _cb

    @staticmethod
    def _transform_stamped_to_transform(msg: TransformStamped) -> Transform:
        p = msg.transform.translation
        o = msg.transform.rotation
        return Transform(t=[p.x, p.y, p.z], q=[o.x, o.y, o.z, o.w])

    def _try_publish(self) -> None:
        if self._published:
            return
        if self._parent.ext_rot is None or self._child.ext_rot is None:
            return

        parent_world = f'{self.parent_namespace}/{self.world_frame}'
        parent_lidar = f'{self.parent_namespace}/{self.lidar_frame}'
        child_world = f'{self.child_namespace}/{self.world_frame}'
        child_lidar = f'{self.child_namespace}/{self.lidar_frame}'

        try:
            parent_tf = self._tf_buffer.lookup_transform(parent_world, parent_lidar, rclpy.time.Time())
            child_tf = self._tf_buffer.lookup_transform(child_world, child_lidar, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        t_lookup_p = self._transform_stamped_to_transform(parent_tf)
        t_lookup_c = self._transform_stamped_to_transform(child_tf)

        # T_(parent/world -> child/world) =
        #     T_lookup_p · extRot_p · T_LpLc · extRot_c^T · inv(T_lookup_c)
        # extRot_* maps raw lidar coords into the gravity-aligned frame
        # (LIO-SAM's internal convention), bridging Multi_LiCa's raw-frame
        # transform with the TF lookup that returns gravity-aligned poses.
        ext_rot_c_inv = _inverse(self._child.ext_rot)
        t_wp_wc = _compose(
            _compose(
                _compose(
                    _compose(t_lookup_p, self._parent.ext_rot),
                    self.t_lidar_parent_child,
                ),
                ext_rot_c_inv,
            ),
            _inverse(t_lookup_c),
        )

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = f'{self.parent_namespace}/{self.publish_frame}'
        tf_msg.child_frame_id = f'{self.child_namespace}/{self.publish_frame}'
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
            f'{tf_msg.header.frame_id} -> {tf_msg.child_frame_id} '
            f'(t=[{t_wp_wc.t[0]:.3f}, {t_wp_wc.t[1]:.3f}, {t_wp_wc.t[2]:.3f}]).'
        )

        for pub in self._shutdown_pubs:
            pub.publish(Empty())
        self.get_logger().info(
            f'Sent shutdown signal to pointcloud aggregators in '
            f'{self.parent_namespace} and {self.child_namespace}.'
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
