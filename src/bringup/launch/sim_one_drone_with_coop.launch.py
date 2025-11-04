#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # arguments
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

    bringup_share = get_package_share_directory('bringup')

    # 1) include your existing 1-drone sim launch
    sim_one_drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_one_drone.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'drone_name': LaunchConfiguration('drone_name'),
        }.items()
    )

    # 2) start the bridge for this drone
    bridge_config = os.path.join(bringup_share, 'config', 'gazebo_bridge.yaml')
    bridge_node = Node(
        package='ros_ign_bridge',          # change to ros_gz_bridge if that's what you have
        executable='parameter_bridge',
        name='ignition_bridge',
        output='screen',
        parameters=[bridge_config]
    )

    # 3) start the coop manager, tell it to watch this drone
    #    IMPORTANT: pass a single string here, not a list-of-substitutions
    coop_node = Node(
        package='coop',
        executable='coverage_manager',
        name='coverage_manager',
        output='screen',
        parameters=[{
            'drone_names': LaunchConfiguration('drone_name')
        }]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        drone_name_arg,
        sim_one_drone_launch,
        bridge_node,
        coop_node,
    ])
