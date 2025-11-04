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

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1) SLAM (online mapping)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params,
                    {'use_sim_time': use_sim_time}]
    )

    # 2) robot_localization (EKF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params,
                    {'use_sim_time': use_sim_time}]
    )

    # 3) Nav2 core servers
    # These names are the usual ones Nav2 expects; your nav2_params.yaml should have matching sections
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params,
                    {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/model/drone1/cmd_vel')]  # send to our drone’s bridged cmd_vel
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[nav2_params,
                    {'use_sim_time': use_sim_time}]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params,
                    {'use_sim_time': use_sim_time}]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params,
                    {'use_sim_time': use_sim_time}]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params,
                    {'use_sim_time': use_sim_time}]
    )

    # 4) Lifecycle manager to bring the Nav2 nodes up
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_node,
        ekf_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])
