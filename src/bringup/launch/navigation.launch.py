#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')

    nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(bringup_share, 'config', 'slam_params.yaml')
    ekf_params = os.path.join(bringup_share, 'config', 'robot_localization.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    # SLAM toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # robot_localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Nav2 (this assumes nav2_params.yaml is from the zip)
    nav2_bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[nav2_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Typically you'd also start controller_server, planner_server, amcl or map_server etc.
    # If the original zip launch did that, mirror it here, but with bringup/... paths.

    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_node,
        ekf_node,
        nav2_bt,
    ])
