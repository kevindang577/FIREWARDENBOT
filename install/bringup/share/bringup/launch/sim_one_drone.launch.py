#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # --- args ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees.sdf',
        description='World file to load from the sim package'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time'
    )

    drone_name_arg = DeclareLaunchArgument(
        'drone_name',
        default_value='drone1',
        description='Name / namespace for the drone model'
    )

    # --- paths ---
    sim_share = get_package_share_directory('sim')
    description_share = get_package_share_directory('description')
    bringup_share = get_package_share_directory('bringup')

    # build world path properly with substitutions
    world_path = PathJoinSubstitution([
        sim_share,
        'worlds',
        LaunchConfiguration('world')
    ])

    robot_description_path = os.path.join(
        description_share,
        'urdf',
        'parrot.urdf.xacro'
    )

    # 1) start Ignition/Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        # if you only have gz: cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 2) robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[robot_description_path]
    )

    # 3) spawn the drone
    spawn_drone = Node(
        package='ros_ign_gazebo',  # change to ros_gz_sim if that's your install
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('drone_name'),
            '-x', '0', '-y', '0', '-z', '0.2',
            '-file', robot_description_path
        ]
    )

    # 4) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(bringup_share, 'config', 'firewardenbot.rviz')],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        drone_name_arg,
        gazebo,
        rsp,
        spawn_drone,
        rviz,
    ])
