#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SingleDroneCoverage(Node):
    def __init__(self):
        super().__init__('single_drone_coverage')

        # params
        self.declare_parameter('world_size', 20.0)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('scan_topic', '/model/drone1/scan')
        self.declare_parameter('odom_topic', '/model/drone1/odometry')
        self.declare_parameter('cmd_vel_topic', '/model/drone1/cmd_vel')

        world_size = float(self.get_parameter('world_size').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # grid
        self.grid_size = int(world_size / self.resolution)
        self.world_origin_x = -world_size / 2.0
        self.world_origin_y = -world_size / 2.0
        self.coverage = [[0 for _ in range(self.grid_size)] for _ in range(self.grid_size)]

        self.current_pose = None
        self.target_cell = None
        self.got_scan = False
        self.got_odom = False

        # manual log throttle
        self.last_wait_log = 0.0
        self.last_done_log = 0.0

        # subs
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        # pub
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # timer
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('coverage_node (single drone) started')
    # ---------------- callbacks ----------------

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        self.current_pose = (x, y, yaw)
        if not self.got_odom:
            self.got_odom = True
            self.get_logger().info(f'Got first odom: ({x:.2f}, {y:.2f}, {yaw:.2f})')

    def scan_cb(self, msg: LaserScan):
        if self.current_pose is None:
            return

        if not self.got_scan:
            self.got_scan = True
            self.get_logger().info('Got first LaserScan')

        rx, ry, ryaw = self.current_pose

        # mark robot cell
        rgx, rgy = self.world_to_grid(rx, ry)
        if self.in_grid(rgx, rgy):
            self.coverage[rgy][rgx] = 1

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            wx = rx + r * math.cos(ryaw + angle)
            wy = ry + r * math.sin(ryaw + angle)

            gx, gy = self.world_to_grid(wx, wy)
            if self.in_grid(gx, gy):
                self.coverage[gy][gx] = 1

            angle += msg.angle_increment

    # --------------- control loop ---------------
    def control_loop(self):
        now = time.time()

        # wait for both streams
        if not self.got_odom or not self.got_scan:
            if now - self.last_wait_log > 5.0:
                self.get_logger().info('Waiting for odom + scan...')
                self.last_wait_log = now
            return

        if self.current_pose is None:
            return

        # need a target?
        if self.target_cell is None:
            frontier = self.find_frontier_cell()
            if frontier is None:
                if now - self.last_done_log > 5.0:
                    self.get_logger().info('No frontier found — maybe covered.')
                    self.last_done_log = now
                self.cmd_pub.publish(Twist())
                return
            self.target_cell = frontier
            wx, wy = self.grid_to_world(*frontier)
            self.get_logger().info(f'New target cell {frontier} -> world ({wx:.2f}, {wy:.2f})')

        self.goto_target()

    def goto_target(self):
        if self.current_pose is None or self.target_cell is None:
            return

        tx, ty = self.grid_to_world(*self.target_cell)
        x, y, yaw = self.current_pose

        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)

        # close enough
        if dist < 0.25:
            self.get_logger().info(f'Reached target ({tx:.2f}, {ty:.2f}), picking new one')
            self.target_cell = None
            self.cmd_pub.publish(Twist())
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = self.angle_diff(target_yaw, yaw)

        cmd = Twist()
        cmd.angular.z = 1.2 * yaw_err
        if abs(yaw_err) < 0.6:
            cmd.linear.x = min(0.6, dist)

        self.cmd_pub.publish(cmd)
    # --------------- frontier search ---------------
    def find_frontier_cell(self):
        if self.current_pose is None:
            return None

        rx, ry, _ = self.current_pose
        rgx, rgy = self.world_to_grid(rx, ry)

        best = None
        best_d2 = 1e18
        min_cells = 3

        for gy in range(self.grid_size):
            row = self.coverage[gy]
            for gx in range(self.grid_size):
                if row[gx] != 1:
                    continue
                if not self.has_unseen_neighbor(gx, gy):
                    continue

                d2 = (gx - rgx) ** 2 + (gy - rgy) ** 2
                if d2 < min_cells * min_cells:
                    continue

                if d2 < best_d2:
                    best_d2 = d2
                    best = (gx, gy)

        return best

    def has_unseen_neighbor(self, gx, gy):
        neighbors = (
            (gx + 1, gy),
            (gx - 1, gy),
            (gx, gy + 1),
            (gx, gy - 1),
        )
        for nx, ny in neighbors:
            if self.in_grid(nx, ny) and self.coverage[ny][nx] == 0:
                return True
        return False

    # --------------- utils ---------------
    def world_to_grid(self, x, y):
        gx = int((x - self.world_origin_x) / self.resolution)
        gy = int((y - self.world_origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.resolution + self.world_origin_x
        y = gy * self.resolution + self.world_origin_y
        return x, y

    def in_grid(self, gx, gy):
        return 0 <= gx < self.grid_size and 0 <= gy < self.grid_size

    @staticmethod
    def quat_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = SingleDroneCoverage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
