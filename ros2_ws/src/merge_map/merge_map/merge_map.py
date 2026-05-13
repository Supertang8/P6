# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

def resample_map(src_map, target_resolution):
    """Resample a map to a target resolution using nearest-neighbor interpolation."""
    src_res = src_map.info.resolution
    src_w = src_map.info.width
    src_h = src_map.info.height
    src_ox = src_map.info.origin.position.x
    src_oy = src_map.info.origin.position.y
    
    if target_resolution <= 0.0:
        raise ValueError('Target resolution must be > 0')
    
    # Compute bounds
    src_x_max = src_ox + src_w * src_res
    src_y_max = src_oy + src_h * src_res
    
    # Output dimensions at target resolution
    out_w = int(np.ceil((src_x_max - src_ox) / target_resolution))
    out_h = int(np.ceil((src_y_max - src_oy) / target_resolution))
    
    # Resample using nearest-neighbor
    resampled_data = []
    for out_y in range(out_h):
        world_y = src_oy + out_y * target_resolution
        src_y = int(np.floor((world_y - src_oy) / src_res))
        if src_y < 0 or src_y >= src_h:
            resampled_data.extend([-1] * out_w)
            continue
        
        for out_x in range(out_w):
            world_x = src_ox + out_x * target_resolution
            src_x = int(np.floor((world_x - src_ox) / src_res))
            if src_x < 0 or src_x >= src_w:
                resampled_data.append(-1)
            else:
                resampled_data.append(src_map.data[src_y * src_w + src_x])
    
    resampled_map = OccupancyGrid()
    resampled_map.header = src_map.header
    resampled_map.info.resolution = target_resolution
    resampled_map.info.width = out_w
    resampled_map.info.height = out_h
    resampled_map.info.origin.position.x = src_ox
    resampled_map.info.origin.position.y = src_oy
    resampled_map.info.origin.position.z = src_map.info.origin.position.z
    resampled_map.info.origin.orientation = src_map.info.origin.orientation
    resampled_map.data = resampled_data
    return resampled_map

def merge_maps(map1, map2):
    merged_map = OccupancyGrid()
    merged_map.header = map1.header
    merged_map.header.frame_id = map1.header.frame_id or map2.header.frame_id
    
    # Determine target resolution (finer of the two)
    target_resolution = min(map1.info.resolution, map2.info.resolution)
    
    # Resample both maps to target resolution if needed
    if map1.info.resolution != target_resolution:
        map1 = resample_map(map1, target_resolution)
    if map2.info.resolution != target_resolution:
        map2 = resample_map(map2, target_resolution)
    
    min_x = min(map1.info.origin.position.x, map2.info.origin.position.x)
    min_y = min(map1.info.origin.position.y, map2.info.origin.position.y)
    max_x = max(map1.info.origin.position.x + (map1.info.width * map1.info.resolution),
                map2.info.origin.position.x + (map2.info.width * map2.info.resolution))
    max_y = max(map1.info.origin.position.y + (map1.info.height * map1.info.resolution),
                map2.info.origin.position.y + (map2.info.height * map2.info.resolution))
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = target_resolution
    if merged_map.info.resolution <= 0.0:
        raise ValueError('Map resolution must be > 0')

    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))

    out_w = merged_map.info.width
    out_h = merged_map.info.height
    out_res = merged_map.info.resolution
    merged_data = [-1] * (out_w * out_h)

    def copy_into_output(src_map, overwrite_unknown_only):
        src_w = src_map.info.width
        src_h = src_map.info.height
        src_res = src_map.info.resolution
        src_ox = src_map.info.origin.position.x
        src_oy = src_map.info.origin.position.y

        for y in range(src_h):
            src_row = y * src_w
            world_y = src_oy + y * src_res
            out_y = int(np.floor((world_y - min_y) / out_res))
            if out_y < 0 or out_y >= out_h:
                continue

            out_row = out_y * out_w
            for x in range(src_w):
                world_x = src_ox + x * src_res
                out_x = int(np.floor((world_x - min_x) / out_res))
                if out_x < 0 or out_x >= out_w:
                    continue

                src_i = src_row + x
                out_i = out_row + out_x

                if overwrite_unknown_only:
                    if merged_data[out_i] == -1:
                        merged_data[out_i] = src_map.data[src_i]
                else:
                    merged_data[out_i] = src_map.data[src_i]

    copy_into_output(map1, overwrite_unknown_only=False)
    copy_into_output(map2, overwrite_unknown_only=True)

    merged_map.data = merged_data
    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        self.subscription_map1 = self.create_subscription(
            OccupancyGrid, '/map1', self.map1_callback, 10)
        self.subscription_map2 = self.create_subscription(
            OccupancyGrid, '/map2', self.map2_callback, 10)
        self.map1 = None
        self.map2 = None
        self.get_logger().info('merge_map_node subscribed to /map1 and /map2')

    def map1_callback(self, msg):
        try:
            self.map1 = msg
            self.get_logger().debug('Received map1')
            if self.map2 is not None:
                msg = merge_maps(self.map1, self.map2)
                self.publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'map1_callback merge failed: {exc}')
    
    def map2_callback(self, msg):
        try:
            self.map2 = msg
            self.get_logger().debug('Received map2')
            if self.map1 is not None:
                msg = merge_maps(self.map1, self.map2)
                self.publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'map2_callback merge failed: {exc}')

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
