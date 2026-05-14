#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

class firstPointInPath(Node):
    def __init__(self):
        super().__init__('first_point_in_path')

        self.map: OccupancyGrid = None
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback, 
            10 
        )

        self.path_subscription = self.create_subscription(
            Path,
            '/plan',
            self.path_callback, 
            10 
        )
        self.publisher = self.create_publisher(PoseStamped, '/offboard/setpoint',10)

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg
    

    def world_to_grid(self, x: float, y: float):
        """"Convert world coordinates to occupancy grid"""
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height

        col = int((x - origin.x) / resolution)
        row = int((y - origin.y) / resolution)

        #return None if out of bounds
        if col < 0 or col >= width or row < 0 or row >= height:
            return None
        
        return col, row

    def get_cell_value(self, col: int, row: int)-> int:
        """Get the occupancy value of a cell in the grid"""
        index = row * self.map.info.width + col
        return self.map.data[index]


    def path_callback(self, msg: Path):
        poses = msg.poses

        if self.map is None:
            return
    
        if not poses:
            return

        for i, pose_stamped in enumerate(poses):
            pos = pose_stamped.pose.position
            cell = self.world_to_grid(pos.x, pos.y)
        
            if cell is None:
                self.get_logger().warn(
                    f'  Pose {i} at ({pos.x:.2f}, {pos.y:.2f}) is outside the map bounds, skipping.'
                )
                continue

            col, row = cell
            cell_value = self.get_cell_value(col, row)

            if cell_value == -1:
            
                setpoint = PoseStamped()
                setpoint.header.stamp = self.get_clock().now().to_msg()
                setpoint.header.frame_id = 'map'
                setpoint.pose.position.x = pos.x
                setpoint.pose.position.y = pos.y
                setpoint.pose.position.z = pos.z
                setpoint.pose.orientation.w = 1.0  # Identity quaternion (no rotation)
                self.publisher.publish(setpoint)
                #print the setpoint
                self.get_logger().info(f'Publishing setpoint: x={setpoint.pose.position.x:.2f}, y={setpoint.pose.position.y:.2f}, z={setpoint.pose.position.z:.2f}')
                return

        
    
def main(args=None):
    rclpy.init(args=args)
    node = firstPointInPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    
