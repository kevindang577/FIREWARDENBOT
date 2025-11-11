#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        self.client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for save_map service...')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Save map service not available')
            return
            
        self.get_logger().info('Save map service found, attempting to save map...')
        self.save_map()
        
    def save_map(self):
        request = SaveMap.Request()
        request.name.data = '/home/student/git/FIREWARDENBOT/demo_map'
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            if future.result().result == 1:
                self.get_logger().info('Map saved successfully!')
                self.get_logger().info('Map files should be saved as demo_map.pgm and demo_map.yaml')
            else:
                self.get_logger().error(f'Failed to save map: {future.result().result}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
