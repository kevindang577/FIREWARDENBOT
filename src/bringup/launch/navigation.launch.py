#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node

def _topic(drone_name_cfg, suffix):
    """
    Make a valid Python expression that evaluates to '/model/<name><suffix>'
    Example: _topic(dn, '/odometry') -> '/model/drone1/odometry'
    """
    return PythonExpression([
        TextSubstitution(text="'/model/' + '"),
        drone_name_cfg,
        TextSubstitution(text="' + '"),
        TextSubstitution(text=suffix),
        TextSubstitution(text="'")
    ])

def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')

    nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(bringup_share, 'config', 'slam_params.yaml')
    ekf_params  = os.path.join(bringup_share, 'config', 'robot_localization.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    drone_name_arg   = DeclareLaunchArgument('drone_name',   default_value='drone1')

    dn = LaunchConfiguration('drone_name')

    # SLAM toolbox (mapping mode) -- remap /scan -> /model/<name>/scan
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/scan', _topic(dn, '/scan')),
        ],
    )

    # robot_localization EKF -- remap odometry & imu inputs to bridged topics
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('odometry', _topic(dn, '/odometry')),
            ('imu',      _topic(dn, '/imu')),
        ],
    )

    # ===== Nav2 core servers =====
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    controller_server = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server', name='smoother_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'smoother_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        drone_name_arg,
        slam_toolbox_node,
        ekf_node,
        planner_server,
        controller_server,
        behavior_server,
        smoother_server,
        waypoint_follower,
        velocity_smoother,
        bt_navigator,
        lifecycle_manager,
    ])
