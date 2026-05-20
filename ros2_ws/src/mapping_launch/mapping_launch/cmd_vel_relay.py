#!/usr/bin/env python3
"""Cross-RMW relay for cmd_vel via UDP loopback.

ROS2 RMW selection is per-process: one process = one RMW. To forward a
topic from one RMW to another on the same host, run this script twice
with different RMW_IMPLEMENTATION env vars and bridge the two halves
over a localhost UDP datagram socket. UDP drops on overflow instead of
blocking, so a slow consumer cannot stall the rclpy executor on the in
side (the bug in the previous SOCK_STREAM version).

Each side can use its own topic name, so you can remap nav2's output to
something neutral and avoid cross-RMW name collisions:
  in subscribes  /cmd_vel_in   (Cyclone)
  out publishes  /cmd_vel      (FastDDS)
Either start order works; UDP is connectionless.

Usage:
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=... \
      ros2 run mapping_launch cmd_vel_relay --side=in  --in-topic /cmd_vel_in
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      ros2 run mapping_launch cmd_vel_relay --side=out --out-topic /cmd_vel
"""
import argparse
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import Twist


DEFAULT_PORT = 38901  # arbitrary unprivileged loopback port
MAX_DGRAM = 65507     # max UDP payload


class InNode(Node):
    """Cyclone-side: subscribe in_topic, ship CDR bytes via UDP loopback."""

    def __init__(self, in_topic: str, port: int):
        super().__init__('cmd_vel_relay_in')
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = ('127.0.0.1', port)
        self.create_subscription(Twist, in_topic, self._cb, 10)
        self.get_logger().info(
            f'in: subscribed {in_topic} (Cyclone) -> UDP {self._addr[0]}:{self._addr[1]}'
        )

    def _cb(self, msg: Twist) -> None:
        data = serialize_message(msg)
        if len(data) > MAX_DGRAM:
            self.get_logger().error(
                f'serialized msg too big for UDP ({len(data)} B); skipping'
            )
            return
        try:
            self._sock.sendto(data, self._addr)
        except OSError as e:
            self.get_logger().warn(
                f'UDP send failed: {e}', throttle_duration_sec=1.0
            )


class OutNode(Node):
    """FastDDS-side: receive CDR bytes via UDP, publish out_topic."""

    def __init__(self, out_topic: str, port: int):
        super().__init__('cmd_vel_relay_out')
        self.pub = self.create_publisher(Twist, out_topic, 10)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1', port))
        self._sock = sock
        self.get_logger().info(
            f'out: publishing {out_topic} (FastDDS) <- UDP 127.0.0.1:{port}'
        )
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        while rclpy.ok():
            try:
                data, _ = self._sock.recvfrom(MAX_DGRAM)
            except OSError:
                return
            try:
                msg = deserialize_message(data, Twist)
            except Exception as e:
                self.get_logger().error(f'deserialize failed: {e}')
                continue
            self.pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description='Cross-RMW cmd_vel relay (UDP)')
    parser.add_argument('--side', choices=['in', 'out'], required=True)
    parser.add_argument('--in-topic', default='/cmd_vel_in',
                        help='Cyclone-side subscribe topic (in side only)')
    parser.add_argument('--out-topic', default='/cmd_vel',
                        help='FastDDS-side publish topic (out side only)')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT,
                        help='UDP loopback port for the bridge')
    args, _ = parser.parse_known_args()

    rclpy.init()
    if args.side == 'in':
        node = InNode(args.in_topic, args.port)
    else:
        node = OutNode(args.out_topic, args.port)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
