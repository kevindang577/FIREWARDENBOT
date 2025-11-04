#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # launch args
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

    drone1_name_arg = DeclareLaunchArgument(
        'drone1_name',
        default_value='drone1',
        description='Name for the first drone'
    )

    drone2_name_arg = DeclareLaunchArgument(
        'drone2_name',
        default_value='drone2',
        description='Name for the second drone'
    )

    # package paths
    sim_share = get_package_share_directory('sim')
    description_share = get_package_share_directory('description')

    # ❗️THIS was the problem line before.
    # You can’t do os.path.join(..., LaunchConfiguration('world'))
    # Use PathJoinSubstitution so launch can resolve it.
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

    # 1) start Ignition/Gazebo once
    gazebo = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r',
            world_path
        ],
        output='screen'
    )

    # 2) robot_state_publisher for each drone (namespaced)
    drone1_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('drone1_name'),
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[robot_description_path]
    )

    drone2_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('drone2_name'),
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[robot_description_path]
    )

    # 3) spawn both drones into Ignition, different poses
    # this part in your original file was already fine
    drone1_spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('drone1_name'),
            '-x', '0', '-y', '0', '-z', '0.2',
            '-file', robot_description_path
        ]
    )

    drone2_spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('drone2_name'),
            '-x', '2.0', '-y', '0', '-z', '0.2',
            '-file', robot_description_path
        ]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        drone1_name_arg,
        drone2_name_arg,
        gazebo,
        drone1_rsp,
        drone2_rsp,
        drone1_spawn,
        drone2_spawn,
    ])
