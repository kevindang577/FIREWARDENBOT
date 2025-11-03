from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    # -----------------------
    # Args
    # -----------------------
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='true'
    ))

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    # -----------------------
    # Robot description (drone)
    # -----------------------
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher)

    # -----------------------
    # robot_localization
    # -----------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    ld.add_action(ekf_node)

    # -----------------------
    # Gazebo world
    # -----------------------
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'),
            'launch',
            'ign_gazebo.launch.py'
        ]),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([pkg_path, 'worlds', 'large_demo.sdf']),
                ' -r'
            ]
        }.items()
    )
    ld.add_action(gazebo)

    # -----------------------
    # Spawn drone
    # -----------------------
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'parrot', '-z', '2.0'],
        output='screen'
    )
    ld.add_action(spawn_robot)

    # -----------------------
    # Gazebo bridge
    # -----------------------
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(bridge)

    # -----------------------
    # SLAM Toolbox
    # -----------------------
    slam_toolbox = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('slam_toolbox'),
            'launch',
            'online_async_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
        }.items()
    )
    ld.add_action(slam_toolbox)

    # -----------------------
    # Nav2 (drone params)
    # -----------------------
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_drone.yaml'])
        }.items()
    )
    ld.add_action(nav2)

    # -----------------------
    # RViz (optional)
    # -----------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=rviz
    )
    ld.add_action(rviz_node)

    # (optional) explorer stays the same as yours
    explorer = Node(
        package='nav2_explore',
        executable='explore_node',
        name='explore',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'planner_frequency': 1.0,
            'progress_timeout': 30.0,
            'visualize': True
        }]
    )
    ld.add_action(explorer)

    return ld
