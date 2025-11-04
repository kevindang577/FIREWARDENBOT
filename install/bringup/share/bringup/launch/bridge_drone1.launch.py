#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    bridge_config = os.path.join(bringup_share, 'config', 'gazebo_bridge.yaml')

    bridge_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ignition_bridge',
        output='screen',
        parameters=[bridge_config]
    )

    return LaunchDescription([
        bridge_node
    ])
