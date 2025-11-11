#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Teleop twist keyboard for drone control  
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            remappings=[
                ('cmd_vel', '/cmd_vel')
            ],
            parameters=[{
                'stamped': False,
                'frame_id': 'base_link'
            }]
        ),
    ])
