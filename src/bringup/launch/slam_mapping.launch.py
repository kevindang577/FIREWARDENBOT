#!/usr/bin/env python3

"""
SLAM Mapping Launch File for Fire Warden Bot
Launches SLAM Toolbox for real-time mapping using LiDAR data
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('bringup'),
            'config',
            'slam_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('scan', '/model/drone1/scan'),
            ('odom', '/model/drone1/odometry')
        ]
    )

    # RViz node for visualization
    rviz_config_dir = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'rviz',
        'slam_config.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        slam_toolbox_node,
        # rviz_node,  # Comment out if RViz is already running
    ])
