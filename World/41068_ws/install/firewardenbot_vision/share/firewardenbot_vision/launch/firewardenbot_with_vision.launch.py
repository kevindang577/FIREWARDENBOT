"""
Integration launch file for Fire Warden Bot with Computer Vision
Integrates vision system with existing 41068_ignition_bringup worlds
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='sparse_forest',
        description='World to launch',
        choices=['simple_trees', 'large_demo', 'sparse_forest', 'dense_forest', 
                'mixed_terrain', 'open_meadows', 'obstacle_course']
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='husky',
        description='Type of robot: husky or drone',
        choices=['husky', 'drone']
    )
    
    enable_vision_arg = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Enable computer vision system'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    enable_nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Enable Nav2 navigation'
    )
    
    # Get configurations
    world = LaunchConfiguration('world')
    robot_type = LaunchConfiguration('robot_type')
    enable_vision = LaunchConfiguration('enable_vision')
    enable_rviz = LaunchConfiguration('rviz')
    enable_nav2 = LaunchConfiguration('nav2')
    
    # Launch the original world with robot
    world_launch = GroupAction([
        # Husky launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('41068_ignition_bringup'),
                    'launch',
                    '41068_ignition.launch.py'
                ])
            ]),
            launch_arguments={
                'world': world,
                'rviz': enable_rviz,
                'nav2': enable_nav2
            }.items(),
            condition=IfCondition(['$(eval "\'', robot_type, '\' == \'husky\'")'])
        ),
        
        # Drone launch  
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('41068_ignition_bringup'),
                    'launch',
                    '41068_ignition_drone.launch.py'
                ])
            ]),
            launch_arguments={
                'world': world,
                'rviz': enable_rviz,
                'nav2': enable_nav2
            }.items(),
            condition=IfCondition(['$(eval "\'', robot_type, '\' == \'drone\'")'])
        )
    ])
    
    # Launch computer vision system
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('firewardenbot_vision'),
                'launch',
                'vision_system.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_type': robot_type,
            'enable_fire_detection': 'true',
            'enable_object_detection': 'true',
            'enable_stereo_vision': 'false',  # Disable by default for performance
            'use_cpp_detector': 'true'
        }.items(),
        condition=IfCondition(enable_vision)
    )
    
    # ROS Bridge for connecting to UI
    ros_bridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'
        }],
        condition=IfCondition(enable_vision)
    )
    
    # Vision data aggregator (combines detections from multiple sources)
    vision_aggregator = Node(
        package='firewardenbot_vision',
        executable='vision_aggregator.py',
        name='vision_aggregator',
        parameters=[{
            'robot_type': robot_type,
            'aggregate_detections': True,
            'publish_combined_alerts': True
        }],
        condition=IfCondition(enable_vision)
    )
    
    return LaunchDescription([
        world_arg,
        robot_type_arg,
        enable_vision_arg,
        enable_rviz_arg,
        enable_nav2_arg,
        world_launch,
        vision_launch,
        ros_bridge_node,
        vision_aggregator
    ])
