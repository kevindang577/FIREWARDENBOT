#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_drone)
        self.start_time = time.time()
        self.get_logger().info('Drone controller started - moving in patterns to generate map data')
        
    def move_drone(self):
        msg = Twist()
        current_time = time.time() - self.start_time
        
        # Create a circular/figure-8 movement pattern
        if current_time < 30:  # First 30 seconds - move forward slowly
            msg.linear.x = 0.5
            msg.angular.z = 0.0
        elif current_time < 60:  # Next 30 seconds - turn and move
            msg.linear.x = 0.3
            msg.angular.z = 0.5 * math.sin(current_time * 0.1)
        elif current_time < 90:  # Next 30 seconds - figure-8 pattern
            msg.linear.x = 0.4
            msg.angular.z = 0.8 * math.sin(current_time * 0.2)
        else:  # Reset pattern
            self.start_time = time.time()
            
        self.publisher.publish(msg)
        
        # Log position periodically
        if int(current_time) % 5 == 0 and current_time - int(current_time) < 0.1:
            self.get_logger().info(f'Moving drone - time: {current_time:.1f}s, linear.x: {msg.linear.x:.2f}, angular.z: {msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
