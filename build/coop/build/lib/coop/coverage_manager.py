#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from typing import Dict, Optional, List


class DroneState:
    """Holds the latest messages for one drone."""
    def __init__(self, name: str):
        self.name = name
        self.odom: Optional[Odometry] = None
        self.scan: Optional[LaserScan] = None


class CoverageManager(Node):
    """
    High-level cooperative manager.

    Right now:
    - reads odom + scan from every drone
    - keeps the most recent messages
    - is ready for coverage/task logic
    """

    def __init__(self):
        super().__init__('coverage_manager')

        # declare params so we can run with:
        #   ros2 run coop coverage_manager --ros-args -p drone_names:="['drone1','drone2']"
        self.declare_parameter('drone_names', ['drone1'])

        drone_names_param = self.get_parameter('drone_names').get_parameter_value()
        # rclpy gives us a ParameterValue; convert to python list
        if drone_names_param.type_ == drone_names_param.TYPE_STRING_ARRAY:
            drone_names: List[str] = list(drone_names_param.string_array_value)
        else:
            # fallback to single-drone
            drone_names = ['drone1']

        self.get_logger().info(f'Coverage manager starting for drones: {drone_names}')

        # store per-drone states
        self._drones: Dict[str, DroneState] = {}

        # create subscriptions for each drone
        for drone_name in drone_names:
            self._create_drone_subscriptions(drone_name)

        # later we can add a timer that checks coverage / assigns tasks
        self._timer = self.create_timer(1.0, self._tick)

    def _create_drone_subscriptions(self, drone_name: str):
        """Set up subscribers for one drone using the model/<name>/... topics."""
        state = DroneState(drone_name)
        self._drones[drone_name] = state

        # topics must match gazebo + bridge
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
        # keep logs light but still helpful
        self.get_logger().debug(f'Received odom from {drone_name}')

    def _scan_cb(self, msg: LaserScan, drone_name: str):
        state = self._drones[drone_name]
        state.scan = msg
        self.get_logger().debug(f'Received scan from {drone_name}')

    def _tick(self):
        """
        Periodic callback.
        For now it just tells us which drones are alive and have data.
        Later this is where we run cooperative coverage.
        """
        info_bits = []
        for name, state in self._drones.items():
            has_odom = state.odom is not None
            has_scan = state.scan is not None
            info_bits.append(f'{name}(odom={has_odom}, scan={has_scan})')
        self.get_logger().info('Drone data status: ' + ', '.join(info_bits))


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
