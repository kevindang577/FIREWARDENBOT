#!/usr/bin/env python3
import math
from typing import Dict, Optional, List, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from coop.task_allocator import GreedyTaskAllocator
from coop.drone_agent import DroneAgent


class DroneState:
    """Holds the latest messages for one drone."""
    def __init__(self, name: str):
        self.name = name
        self.odom: Optional[Odometry] = None
        self.scan: Optional[LaserScan] = None

    def position_xy(self) -> Optional[Tuple[float, float]]:
        if self.odom is None:
            return None
        pose = self.odom.pose.pose
        return (pose.position.x, pose.position.y)


class CoverageManager(Node):
    """
    High-level cooperative manager.

    Now it actually:
    - tracks a shared coverage grid
    - marks coverage around each drone
    - allocates unvisited spots to idle drones
    - stops when coverage is above a threshold
    """

    def __init__(self):
        super().__init__('coverage_manager')

        # params
        self.declare_parameter('drone_names', ['drone1'])
        self.declare_parameter('map_width', 40.0)     # meters
        self.declare_parameter('map_height', 40.0)    # meters
        self.declare_parameter('map_resolution', 0.5) # meters / cell
        self.declare_parameter('map_origin_x', -20.0)
        self.declare_parameter('map_origin_y', -20.0)
        self.declare_parameter('coverage_radius', 1.5)  # meters around drone to mark visited
        self.declare_parameter('coverage_complete_threshold', 0.95)  # 95%

        # load basic params
        drone_names_param = self.get_parameter('drone_names').get_parameter_value()
        if drone_names_param.type_ == drone_names_param.TYPE_STRING_ARRAY:
            drone_names: List[str] = list(drone_names_param.string_array_value)
        else:
            drone_names = ['drone1']

        self._map_width = float(self.get_parameter('map_width').value)
        self._map_height = float(self.get_parameter('map_height').value)
        self._map_resolution = float(self.get_parameter('map_resolution').value)
        self._origin_x = float(self.get_parameter('map_origin_x').value)
        self._origin_y = float(self.get_parameter('map_origin_y').value)
        self._coverage_radius = float(self.get_parameter('coverage_radius').value)
        self._coverage_complete_threshold = float(self.get_parameter('coverage_complete_threshold').value)

        # derived
        self._cells_x = max(1, int(self._map_width / self._map_resolution))
        self._cells_y = max(1, int(self._map_height / self._map_resolution))

        self.get_logger().info(
            f'Coverage grid: {self._cells_x} x {self._cells_y} cells '
            f'@ {self._map_resolution} m, origin=({self._origin_x}, {self._origin_y})'
        )

        # 2D coverage grid: [y][x] = bool
        self._coverage_grid: List[List[bool]] = [
            [False for _ in range(self._cells_x)] for _ in range(self._cells_y)
        ]
        self._covered_cells = 0
        self._total_cells = self._cells_x * self._cells_y

        # per-drone states
        self._drones: Dict[str, DroneState] = {}
        self._agents: Dict[str, DroneAgent] = {}

        # allocator
        self._allocator = GreedyTaskAllocator(
            origin_x=self._origin_x,
            origin_y=self._origin_y,
            resolution=self._map_resolution,
            cells_x=self._cells_x,
            cells_y=self._cells_y,
        )

        self.get_logger().info(f'Coverage manager starting for drones: {drone_names}')

        # create subscriptions + agents
        for drone_name in drone_names:
            self._create_drone_subscriptions(drone_name)
            self._agents[drone_name] = DroneAgent(self, drone_name)

        # main loop
        self._timer = self.create_timer(1.0, self._tick)

    # -----------------------------------
    # subscriptions
    # -----------------------------------
    def _create_drone_subscriptions(self, drone_name: str):
        state = DroneState(drone_name)
        self._drones[drone_name] = state

        odom_topic = f'/model/{drone_name}/odometry'
        scan_topic = f'/model/{drone_name}/scan'

        self.get_logger().info(f'Subscribing to {odom_topic} and {scan_topic}')

        self.create_subscription(
            Odometry,
            odom_topic,
            lambda msg, dn=drone_name: self._odom_cb(msg, dn),
            10
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            lambda msg, dn=drone_name: self._scan_cb(msg, dn),
            10
        )

    def _odom_cb(self, msg: Odometry, drone_name: str):
        state = self._drones[drone_name]
        state.odom = msg

    def _scan_cb(self, msg: LaserScan, drone_name: str):
        state = self._drones[drone_name]
        state.scan = msg

    # -----------------------------------
    # coverage helpers
    # -----------------------------------
    def _world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """
        Convert world (map) coordinates to grid cell indices.
        Returns None if outside grid.
        """
        ix = int((x - self._origin_x) / self._map_resolution)
        iy = int((y - self._origin_y) / self._map_resolution)
        if ix < 0 or iy < 0 or ix >= self._cells_x or iy >= self._cells_y:
            return None
        return ix, iy

    def _mark_coverage_disc(self, x: float, y: float):
        """
        Mark cells within coverage_radius of (x, y) as visited.
        """
        cell = self._world_to_cell(x, y)
        if cell is None:
            return
        cx, cy = cell

        radius_cells = int(self._coverage_radius / self._map_resolution)
        for iy in range(cy - radius_cells, cy + radius_cells + 1):
            if iy < 0 or iy >= self._cells_y:
                continue
            for ix in range(cx - radius_cells, cx + radius_cells + 1):
                if ix < 0 or ix >= self._cells_x:
                    continue
                # circle-ish mask
                dx = ix - cx
                dy = iy - cy
                if (dx * dx + dy * dy) * (self._map_resolution ** 2) <= self._coverage_radius * self._coverage_radius:
                    if not self._coverage_grid[iy][ix]:
                        self._coverage_grid[iy][ix] = True
                        self._covered_cells += 1

    def _compute_coverage_ratio(self) -> float:
        if self._total_cells == 0:
            return 1.0
        return float(self._covered_cells) / float(self._total_cells)

    # -----------------------------------
    # main tick
    # -----------------------------------
    def _tick(self):
        # 1. update coverage from each drone
        for name, state in self._drones.items():
            pos = state.position_xy()
            if pos is not None:
                self._mark_coverage_disc(pos[0], pos[1])

        # 2. check overall coverage
        coverage = self._compute_coverage_ratio()
        self.get_logger().info(f'Coverage: {coverage * 100.0:.1f}%')

        if coverage >= self._coverage_complete_threshold:
            self.get_logger().info('Coverage complete. No more goals will be assigned.')
            # we could tell agents to cancel, but if they've reached their goal they'll just idle
            return

        # 3. figure out which drones are idle and have a pose
        idle_drones: Dict[str, Tuple[float, float]] = {}
        for name, state in self._drones.items():
            pos = state.position_xy()
            agent = self._agents[name]
            if pos is not None and not agent.is_busy():
                idle_drones[name] = pos

        if not idle_drones:
            # everyone is moving or we don't have poses yet
            return

        # 4. ask allocator for assignments
        assignments = self._allocator.assign(self._coverage_grid, idle_drones)

        # 5. send goals
        for name, goal in assignments.items():
            agent = self._agents[name]
            sent = agent.send_goal(goal[0], goal[1], yaw=0.0)
            if not sent:
                self.get_logger().warn(f'Failed to send goal to {name}')

def main(args=None):
    rclpy.init(args=args)
    node = CoverageManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
