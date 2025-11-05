#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarReactive(Node):
    def __init__(self):
        super().__init__('lidar_reactive')

        self.declare_parameter('scan_topic', '/model/drone1/scan')
        self.declare_parameter('cmd_vel_topic', '/model/drone1/cmd_vel')
        self.declare_parameter('obstacle_distance', 2.0)
        self.declare_parameter('forward_speed', 0.5)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('front_angle_deg', 60.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))

        self.last_scan = None

        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('lidar_reactive started')

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def control_loop(self):
        cmd = Twist()

        if self.last_scan is None:
            self.cmd_pub.publish(cmd)
            return

        msg = self.last_scan

        half = self.front_angle / 2.0
        front_min = -half
        front_max = half

        min_front = None
        angle = msg.angle_min
        for r in msg.ranges:
            if front_min <= angle <= front_max:
                if not math.isinf(r) and not math.isnan(r):
                    if min_front is None or r < min_front:
                        min_front = r
            angle += msg.angle_increment

        if min_front is not None and min_front < self.obstacle_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarReactive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
