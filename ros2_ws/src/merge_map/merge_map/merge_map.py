# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

from tf2_ros import Buffer, TransformListener, TransformException


OCCUPIED_THRESH = 50  # projected_map values >= this are treated as occupied


def resample_map(src_map, target_resolution):
    """Resample a map to a target resolution using nearest-neighbor interpolation."""
    src_res = src_map.info.resolution
    src_w = src_map.info.width
    src_h = src_map.info.height
    src_ox = src_map.info.origin.position.x
    src_oy = src_map.info.origin.position.y

    if target_resolution <= 0.0:
        raise ValueError('Target resolution must be > 0')

    src_x_max = src_ox + src_w * src_res
    src_y_max = src_oy + src_h * src_res

    out_w = int(np.ceil((src_x_max - src_ox) / target_resolution))
    out_h = int(np.ceil((src_y_max - src_oy) / target_resolution))

    src_data = np.array(src_map.data, dtype=np.int8).reshape(src_h, src_w)

    out_y_idx = np.arange(out_h)
    src_y_for_out = np.floor((out_y_idx * target_resolution) / src_res).astype(int)
    src_y_for_out = np.clip(src_y_for_out, 0, src_h - 1)

    out_x_idx = np.arange(out_w)
    src_x_for_out = np.floor((out_x_idx * target_resolution) / src_res).astype(int)
    src_x_for_out = np.clip(src_x_for_out, 0, src_w - 1)

    resampled = src_data[np.ix_(src_y_for_out, src_x_for_out)]

    resampled_map = OccupancyGrid()
    resampled_map.header = src_map.header
    resampled_map.info.resolution = target_resolution
    resampled_map.info.width = out_w
    resampled_map.info.height = out_h
    resampled_map.info.origin.position.x = src_ox
    resampled_map.info.origin.position.y = src_oy
    resampled_map.info.origin.position.z = src_map.info.origin.position.z
    resampled_map.info.origin.orientation = src_map.info.origin.orientation
    resampled_map.data = resampled.flatten().tolist()
    return resampled_map


def _place_into_grid(src_map, out_h, out_w, out_res, min_x, min_y):
    """Place src_map into a (out_h, out_w) int8 array filled with -1 (unknown)."""
    grid = np.full((out_h, out_w), -1, dtype=np.int8)
    src_w = src_map.info.width
    src_h = src_map.info.height
    src_ox = src_map.info.origin.position.x
    src_oy = src_map.info.origin.position.y

    off_x = int(round((src_ox - min_x) / out_res))
    off_y = int(round((src_oy - min_y) / out_res))

    src_data = np.array(src_map.data, dtype=np.int8).reshape(src_h, src_w)

    x_start = max(0, off_x)
    y_start = max(0, off_y)
    x_end = min(out_w, off_x + src_w)
    y_end = min(out_h, off_y + src_h)

    if x_end <= x_start or y_end <= y_start:
        return grid

    sx_start = x_start - off_x
    sy_start = y_start - off_y
    sx_end = sx_start + (x_end - x_start)
    sy_end = sy_start + (y_end - y_start)

    grid[y_start:y_end, x_start:x_end] = src_data[sy_start:sy_end, sx_start:sx_end]
    return grid


def merge_maps(rover_map, drone_map, rover_xy, footprint_radius):
    """Merge rover and drone occupancy grids.

    Policy:
      - Inside a disk of radius `footprint_radius` around `rover_xy` the rover
        map has absolute priority (drone is ignored). This prevents the drone's
        view of the rover-as-obstacle from trapping the rover.
      - Outside the disk: occupied-wins (either source) > free-wins > unknown.

    `rover_xy` may be None, in which case the footprint carve-out is skipped
    and the global occupied-wins / free-wins rule applies everywhere.
    """
    target_resolution = min(rover_map.info.resolution, drone_map.info.resolution)
    if rover_map.info.resolution != target_resolution:
        rover_map = resample_map(rover_map, target_resolution)
    if drone_map.info.resolution != target_resolution:
        drone_map = resample_map(drone_map, target_resolution)

    min_x = min(rover_map.info.origin.position.x, drone_map.info.origin.position.x)
    min_y = min(rover_map.info.origin.position.y, drone_map.info.origin.position.y)
    max_x = max(rover_map.info.origin.position.x + rover_map.info.width * rover_map.info.resolution,
                drone_map.info.origin.position.x + drone_map.info.width * drone_map.info.resolution)
    max_y = max(rover_map.info.origin.position.y + rover_map.info.height * rover_map.info.resolution,
                drone_map.info.origin.position.y + drone_map.info.height * drone_map.info.resolution)

    out_res = target_resolution
    if out_res <= 0.0:
        raise ValueError('Map resolution must be > 0')
    out_w = int(np.ceil((max_x - min_x) / out_res))
    out_h = int(np.ceil((max_y - min_y) / out_res))

    rover_grid = _place_into_grid(rover_map, out_h, out_w, out_res, min_x, min_y)
    drone_grid = _place_into_grid(drone_map, out_h, out_w, out_res, min_x, min_y)

    rover_occ = rover_grid >= OCCUPIED_THRESH
    drone_occ = drone_grid >= OCCUPIED_THRESH
    rover_free = (rover_grid >= 0) & (rover_grid < OCCUPIED_THRESH)
    drone_free = (drone_grid >= 0) & (drone_grid < OCCUPIED_THRESH)

    merged = np.full((out_h, out_w), -1, dtype=np.int8)
    merged[rover_free | drone_free] = 0
    merged[rover_occ | drone_occ] = 100

    if rover_xy is not None and footprint_radius > 0.0:
        rx, ry = rover_xy
        # Cell-center world coordinates.
        xs = min_x + (np.arange(out_w) + 0.5) * out_res
        ys = min_y + (np.arange(out_h) + 0.5) * out_res
        xx, yy = np.meshgrid(xs, ys)
        footprint = (xx - rx) ** 2 + (yy - ry) ** 2 <= footprint_radius ** 2
        merged[footprint] = rover_grid[footprint]

    merged_map = OccupancyGrid()
    merged_map.header = rover_map.header
    merged_map.header.frame_id = rover_map.header.frame_id or drone_map.header.frame_id
    merged_map.info.resolution = out_res
    merged_map.info.width = out_w
    merged_map.info.height = out_h
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.origin.position.z = rover_map.info.origin.position.z
    merged_map.info.origin.orientation = rover_map.info.origin.orientation
    merged_map.data = merged.flatten().tolist()
    return merged_map


class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')

        self.declare_parameter('footprint_radius', 2.0)
        self.declare_parameter('rover_base_frame', 'rover/base_link')
        self.footprint_radius = float(
            self.get_parameter('footprint_radius').value)
        self.rover_base_frame = str(
            self.get_parameter('rover_base_frame').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        self.subscription_map1 = self.create_subscription(
            OccupancyGrid, '/map1', self.map1_callback, 10)
        self.subscription_map2 = self.create_subscription(
            OccupancyGrid, '/map2', self.map2_callback, 10)
        self.map1 = None
        self.map2 = None
        self.get_logger().info(
            f'merge_map_node subscribed to /map1 (rover) and /map2 (drone); '
            f'rover-priority radius={self.footprint_radius} m around '
            f'{self.rover_base_frame}')

    def _lookup_rover_xy(self, target_frame):
        if not target_frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, self.rover_base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'Rover footprint TF lookup failed '
                f'({target_frame} <- {self.rover_base_frame}): {exc}',
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def _try_merge_and_publish(self):
        if self.map1 is None or self.map2 is None:
            return
        rover_xy = self._lookup_rover_xy(self.map1.header.frame_id)
        try:
            msg = merge_maps(self.map1, self.map2,
                             rover_xy, self.footprint_radius)
        except Exception as exc:
            self.get_logger().error(f'merge failed: {exc}')
            return
        self.publisher.publish(msg)

    def map1_callback(self, msg):
        self.map1 = msg
        self._try_merge_and_publish()

    def map2_callback(self, msg):
        self.map2 = msg
        self._try_merge_and_publish()


def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
