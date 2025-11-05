#!/usr/bin/env python3
"""
Leaf Detection Monitor - Displays real-time leaf detection results
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import json
from datetime import datetime

class LeafDetectionMonitor(Node):
    def __init__(self):
        super().__init__('leaf_detection_monitor')
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            String,
            '/leaf_detections',
            self.detection_callback,
            10
        )
        
        self.location_sub = self.create_subscription(
            PointStamped,
            '/leaf_detector/leaf_locations',
            self.location_callback,
            10
        )
        
        # Statistics
        self.total_detections = 0
        self.color_counts = {}
        self.recent_locations = []
        
        self.get_logger().info('Leaf Detection Monitor started - Monitoring for foliage detections...')
        
        # Create a timer to periodically display statistics
        self.timer = self.create_timer(5.0, self.display_statistics)
    
    def detection_callback(self, msg):
        """Process incoming leaf detection data"""
        try:
            data = json.loads(msg.data)
            
            detections = data.get('detections', [])
            if len(detections) > 0:
                self.get_logger().info(f'🍂 NEW LEAF DETECTIONS ({len(detections)} leaves found)')
                
                for i, detection in enumerate(detections):
                    color = detection['color_category']
                    confidence = detection['confidence']
                    area = detection['area_pixels']
                    center = detection['center_pixel']
                    
                    # Update statistics
                    self.total_detections += 1
                    self.color_counts[color] = self.color_counts.get(color, 0) + 1
                    
                    # Display detection details
                    self.get_logger().info(
                        f'  Leaf {i+1}: {color.upper()} '
                        f'(confidence: {confidence:.2f}, '
                        f'area: {area:.0f}px, '
                        f'center: {center})'
                    )
                
                print('-' * 60)
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing detection data: {e}')
    
    def location_callback(self, msg):
        """Process leaf location data"""
        location = (msg.point.x, msg.point.y)
        self.recent_locations.append(location)
        
        # Keep only recent locations (last 10)
        if len(self.recent_locations) > 10:
            self.recent_locations.pop(0)
    
    def display_statistics(self):
        """Display periodic statistics summary"""
        if self.total_detections > 0:
            self.get_logger().info('🌿 LEAF DETECTION SUMMARY:')
            self.get_logger().info(f'   Total leaves detected: {self.total_detections}')
            
            # Color breakdown
            for color, count in sorted(self.color_counts.items()):
                percentage = (count / self.total_detections) * 100
                self.get_logger().info(f'   {color}: {count} leaves ({percentage:.1f}%)')
            
            # Recent locations
            if self.recent_locations:
                self.get_logger().info(f'   Recent locations: {len(self.recent_locations)} points')
            
            print('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    
    monitor = LeafDetectionMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Leaf Detection Monitor shutting down...')
        
        # Final summary
        if monitor.total_detections > 0:
            monitor.get_logger().info('🍁 FINAL DETECTION SUMMARY:')
            monitor.get_logger().info(f'Total leaves found: {monitor.total_detections}')
            for color, count in sorted(monitor.color_counts.items()):
                percentage = (count / monitor.total_detections) * 100
                monitor.get_logger().info(f'{color}: {count} ({percentage:.1f}%)')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
