from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class TfNamespaceRepublisher(Node):
    def __init__(self) -> None:
        super().__init__('tf_namespace_republisher')
        self._target_frames = {'base_link', 'camera_init', 'body', 'odom'}

        tf_static_qos = QoSProfile(depth=1)
        tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._tf_sub = self.create_subscription(TFMessage, '/tf', self._tf_cb, 100)
        self._tf_pub = self.create_publisher(TFMessage, '/tf', 100)

        self._tf_static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self._tf_static_cb,
            tf_static_qos,
        )
        self._tf_static_pub = self.create_publisher(TFMessage, '/tf_static', tf_static_qos)

    def _namespace_prefix(self) -> str:
        namespace = self.get_namespace().strip('/')
        return namespace

    @staticmethod
    def _strip_leading_slash(frame_id: str) -> str:
        return frame_id[1:] if frame_id.startswith('/') else frame_id

    def _prefix_frame_if_target(self, frame_id: str) -> tuple[str, bool]:
        cleaned = self._strip_leading_slash(frame_id)
        if cleaned not in self._target_frames:
            return frame_id, False
        prefix = self._namespace_prefix()
        if not prefix:
            return frame_id, False
        return f'{prefix}/{cleaned}', True

    def _rewrite_message(self, msg: TFMessage) -> TFMessage | None:
        rewritten = TFMessage()
        for transform in msg.transforms:
            updated_transform = deepcopy(transform)
            parent_frame, parent_changed = self._prefix_frame_if_target(updated_transform.header.frame_id)
            child_frame, child_changed = self._prefix_frame_if_target(updated_transform.child_frame_id)

            if parent_changed or child_changed:
                updated_transform.header.frame_id = parent_frame
                updated_transform.child_frame_id = child_frame
                rewritten.transforms.append(updated_transform)

        return rewritten if rewritten.transforms else None

    def _tf_cb(self, msg: TFMessage) -> None:
        rewritten = self._rewrite_message(msg)
        if rewritten is not None:
            self._tf_pub.publish(rewritten)

    def _tf_static_cb(self, msg: TFMessage) -> None:
        rewritten = self._rewrite_message(msg)
        if rewritten is not None:
            self._tf_static_pub.publish(rewritten)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfNamespaceRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()