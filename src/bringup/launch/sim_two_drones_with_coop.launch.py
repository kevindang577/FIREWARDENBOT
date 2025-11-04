#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # common args
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
        description='Name for first drone'
    )

    drone2_name_arg = DeclareLaunchArgument(
        'drone2_name',
        default_value='drone2',
        description='Name for second drone'
    )

    bringup_share = get_package_share_directory('bringup')

    # 1) include the 2-drone sim launch
    sim_two_drones_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_two_drones.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'drone1_name': LaunchConfiguration('drone1_name'),
            'drone2_name': LaunchConfiguration('drone2_name'),
        }.items()
    )

    # 2) start the 2-drone bridge
    bridge_config = os.path.join(bringup_share, 'config', 'gazebo_bridge_two_drones.yaml')

    bridge_node = Node(
        package='ros_ign_bridge',  # change to ros_gz_bridge if that's what you have
        executable='parameter_bridge',
        name='ignition_bridge_two_drones',
        output='screen',
        parameters=[bridge_config]
    )

    # 3) start the coop node, tell it to watch both drones
    coop_node = Node(
        package='coop',
        executable='coverage_manager',
        name='coverage_manager',
        output='screen',
        parameters=[{
            # pass as list of strings
            'drone_names': [
                LaunchConfiguration('drone1_name'),
                LaunchConfiguration('drone2_name')
            ]
        }]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        drone1_name_arg,
        drone2_name_arg,
        sim_two_drones_launch,
        bridge_node,
        coop_node,
    ])
