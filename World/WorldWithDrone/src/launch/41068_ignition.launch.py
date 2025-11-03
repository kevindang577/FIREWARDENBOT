from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # ----------------------------------
    # Launch args
    # ----------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2',
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Launch Nav2 stack',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='modified',
        description='World to load',
        choices=[
            'simple_trees',
            'large_demo',
            'sparse_forest',
            'dense_forest',
            'mixed_terrain',
            'open_meadows',
            'obstacle_course',
            'modified',
        ],
    )

    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(nav2_arg)
    ld.add_action(world_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')

    # ----------------------------------
    # Robot description (DRONE, not husky)
    # ----------------------------------
    drone_xacro = PathJoinSubstitution([
        pkg_path,
        'urdf_drone',
        'parrot.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', drone_xacro]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )
    ld.add_action(robot_state_publisher)

    # ----------------------------------
    # robot_localization (gives odom->base_link etc.)
    # ----------------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time},
        ],
    )
    ld.add_action(ekf_node)

    # ----------------------------------
    # Start Ignition/Gazebo with chosen world
    # ----------------------------------
    world_path = PathJoinSubstitution([
        pkg_path,
        'worlds',
        world_name.perform  # this is how you logically think about it
    ])
    # BUT in launch substitutions we do:
    world_path = PathJoinSubstitution([
        pkg_path,
        'worlds',
        LaunchConfiguration('world') + '.sdf'
    ])

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'),
            'launch',
            'ign_gazebo.launch.py'
        ]),
        launch_arguments={
            'ign_args': [world_path, ' -r']
        }.items()
    )
    ld.add_action(gazebo)

    # ----------------------------------
    # Spawn the drone
    # ----------------------------------
    spawn_drone = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # spawn what robot_state_publisher is publishing
        arguments=['-topic', '/robot_description', '-name', 'drone', '-z', '1.0']
    )
    ld.add_action(spawn_drone)

    # ----------------------------------
    # Bridge (we extended gazebo_bridge.yaml earlier to add /drone/*)
    # ----------------------------------
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )
    ld.add_action(bridge)

    # ----------------------------------
    # RViz (optional)
    # ----------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # ----------------------------------
    # Nav2 (optional) – point this at the drone params we made
    # ----------------------------------
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_drone.yaml']),
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
