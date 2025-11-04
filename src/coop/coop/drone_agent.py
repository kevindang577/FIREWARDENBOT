#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

import math


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    # 2D yaw → quaternion
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class DroneAgent:
    """
    Per-drone navigation adapter.

    This assumes there is a Nav2-like action server available.
    We try /<drone_name>/navigate_to_pose first, and fall back to /navigate_to_pose.
    """

    def __init__(self, node: Node, drone_name: str, action_name: Optional[str] = None):
        self._node = node
        self._drone_name = drone_name

        # try namespaced first
        if action_name is None:
            self._preferred_action_name = f'/{drone_name}/navigate_to_pose'
        else:
            self._preferred_action_name = action_name

        # we will lazily create the client(s)
        self._client: Optional[ActionClient] = None
        self._fallback_client: Optional[ActionClient] = None

        self._busy: bool = False
        self._current_goal = None
        self._last_success = True

        # create the preferred client right away
        self._client = ActionClient(self._node, NavigateToPose, self._preferred_action_name)

        # we don't block forever here; manager ticks will check availability

    def is_busy(self) -> bool:
        return self._busy

    def _ensure_client(self) -> Optional[ActionClient]:
        """
        Make sure we have *some* action server to talk to.
        Returns the client to use, or None if nothing is available.
        """
        # 1) try preferred
        if self._client is not None:
            if self._client.wait_for_server(timeout_sec=0.05):
                return self._client

        # 2) try fallback global
        if self._fallback_client is None:
            # create on demand
            self._fallback_client = ActionClient(self._node, NavigateToPose, '/navigate_to_pose')

        if self._fallback_client.wait_for_server(timeout_sec=0.05):
            return self._fallback_client

        return None

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """
        Ask the drone to go to (x, y, yaw). Returns True if the goal was accepted.
        """
        if self._busy:
            # don't stomp current goal
            return False

        client = self._ensure_client()
        if client is None:
            self._node.get_logger().warn(
                f'[{self._drone_name}] No navigate_to_pose action server available yet.'
            )
            return False

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._node.get_logger().info(
            f'[{self._drone_name}] Sending goal to ({x:.2f}, {y:.2f})'
        )

        send_future = client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

        self._busy = True
        self._current_goal = (x, y, yaw)
        return True

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(f'[{self._drone_name}] Goal was rejected by server.')
            self._busy = False
            return

        self._node.get_logger().debug(f'[{self._drone_name}] Goal accepted, waiting for result.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        # we could log progress here, but keep it quiet for now
        pass

    def _result_cb(self, future):
        result = future.result().result
        # success is 4 in NavigateToPose, but let's just log for now
        self._node.get_logger().info(f'[{self._drone_name}] Goal result: {result}')
        self._busy = False
        self._current_goal = None

    def cancel_current_goal(self):
        # not strictly required for coverage, but nice to have
        # we would need to keep the goal_handle around to cancel;
        # for now, we just mark free, manager will stop assigning.
        self._busy = False
        self._current_goal = None
