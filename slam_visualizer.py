#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import numpy as np

class SLAMVisualizer(Node):
    def __init__(self):
        super().__init__('slam_visualizer')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/model/drone1/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
            
        # Data storage
        self.latest_scan = None
        self.latest_odom = None
        self.latest_map = None
        self.scan_count = 0
        self.odom_count = 0
        
        # Timer to generate visualization
        self.timer = self.create_timer(2.0, self.generate_visualization)
        
        self.get_logger().info('SLAM Visualizer started - monitoring topics...')
        
    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_count += 1
        
    def odom_callback(self, msg):
        self.latest_odom = msg
        self.odom_count += 1
        
    def map_callback(self, msg):
        self.latest_map = msg
        self.get_logger().info('Map received! Creating visualization...')
        
    def generate_visualization(self):
        self.get_logger().info(f'SLAM Status - Scans: {self.scan_count}, Odom: {self.odom_count}')
        
        if self.latest_scan is not None:
            self.get_logger().info(f'Latest scan: {len(self.latest_scan.ranges)} points, range: {self.latest_scan.range_min:.2f}-{self.latest_scan.range_max:.2f}m')
            
        if self.latest_odom is not None:
            pos = self.latest_odom.pose.pose.position
            self.get_logger().info(f'Robot position: x={pos.x:.2f}, y={pos.y:.2f}')
            
        # Create a simple visualization of current scan data
        if self.latest_scan is not None and len(self.latest_scan.ranges) > 0:
            self.create_scan_plot()
            
    def create_scan_plot(self):
        scan = self.latest_scan
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges)
        
        # Filter out invalid readings
        valid_indices = (ranges >= scan.range_min) & (ranges <= scan.range_max)
        valid_angles = angles[valid_indices]
        valid_ranges = ranges[valid_indices]
        
        if len(valid_ranges) == 0:
            self.get_logger().warn('No valid scan data')
            return
            
        # Convert to cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # Create plot
        plt.figure(figsize=(10, 8))
        plt.subplot(211)
        plt.scatter(x, y, s=1, alpha=0.6)
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title(f'Current LiDAR Scan (Point Cloud) - {len(valid_ranges)} points')
        plt.grid(True)
        plt.axis('equal')
        
        # Add robot position if available
        if self.latest_odom is not None:
            plt.plot(0, 0, 'ro', markersize=8, label='Robot')
            plt.legend()
        
        plt.subplot(212)
        plt.plot(valid_angles * 180 / np.pi, valid_ranges)
        plt.xlabel('Angle (degrees)')
        plt.ylabel('Range (meters)')
        plt.title('LiDAR Range Data')
        plt.grid(True)
        
        plt.tight_layout()
        
        # Save plot
        filename = '/home/student/git/FIREWARDENBOT/slam_visualization.png'
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'Visualization saved to: {filename}')
        self.get_logger().info(f'Scan data summary: {len(valid_ranges)} valid points, range {valid_ranges.min():.2f}-{valid_ranges.max():.2f}m')

def main(args=None):
    rclpy.init(args=args)
    visualizer = SLAMVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
