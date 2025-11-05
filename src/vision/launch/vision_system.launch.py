#!/usr/bin/env python3
"""
Launch file for Fire Warden Bot Computer Vision System
Starts leaf detection and classification nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Leaf Detection Node
        Node(
            package='vision',
            executable='leaf_detector',
            name='leaf_detector',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=[
                ('/model/drone1/camera', '/model/drone1/camera'),
                ('/leaf_detections', '/leaf_detections'),
                ('/leaf_detector/annotated_image', '/leaf_detector/annotated_image'),
                ('/leaf_detector/leaf_locations', '/leaf_detector/leaf_locations')
            ]
        ),
        
        # Optional: Image view for monitoring (comment out if not needed)
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='annotated_image_viewer',
        #     arguments=['/leaf_detector/annotated_image']
        # )
    ])
