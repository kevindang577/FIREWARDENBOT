#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np
from PIL import Image
import sys

class MapListener(Node):
    def __init__(self):
        super().__init__('map_listener')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.map_received = False
        self.get_logger().info('Listening for map data on /map topic...')
        
    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}')
            self.save_map(msg)
            self.map_received = True
            
    def save_map(self, occupancy_grid):
        # Save the map data
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        origin = occupancy_grid.info.origin
        
        # Convert occupancy grid data to image
        data = np.array(occupancy_grid.data, dtype=np.int8)
        data = data.reshape((height, width))
        
        # Convert to image format (0-255)
        # -1 (unknown) -> 128 (grey)
        # 0 (free) -> 255 (white) 
        # 100 (occupied) -> 0 (black)
        image_data = np.zeros_like(data, dtype=np.uint8)
        image_data[data == -1] = 128  # Unknown
        image_data[data == 0] = 255   # Free
        image_data[(data > 0) & (data <= 100)] = 0  # Occupied
        
        # Flip vertically (ROS convention vs image convention)
        image_data = np.flipud(image_data)
        
        # Save as PGM image
        img = Image.fromarray(image_data, mode='L')
        map_file = '/home/student/git/FIREWARDENBOT/slam_generated_map.pgm'
        img.save(map_file)
        
        # Save metadata YAML file
        yaml_data = {
            'image': 'slam_generated_map.pgm',
            'resolution': float(resolution),
            'origin': [float(origin.position.x), float(origin.position.y), float(origin.position.z)],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        yaml_file = '/home/student/git/FIREWARDENBOT/slam_generated_map.yaml'
        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
            
        self.get_logger().info(f'Map saved to {map_file} and {yaml_file}')
        self.get_logger().info(f'Map size: {width}x{height}, resolution: {resolution} m/pixel')

def main(args=None):
    rclpy.init(args=args)
    node = MapListener()
    
    try:
        # Run for up to 30 seconds to wait for map data
        import time
        start_time = time.time()
        while rclpy.ok() and not node.map_received and (time.time() - start_time) < 30:
            rclpy.spin_once(node, timeout_sec=1.0)
            
        if node.map_received:
            node.get_logger().info('Map successfully saved!')
        else:
            node.get_logger().error('No map data received within timeout')
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
