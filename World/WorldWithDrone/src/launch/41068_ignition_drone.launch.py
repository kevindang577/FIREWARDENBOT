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

    # --------------------
    # launch args
    # --------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Flag to enable use_sim_time'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Flag to launch RViz'
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Flag to launch Nav2'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=[
            'simple_trees',
            'large_demo',
            'sparse_forest',
            'dense_forest',
            'mixed_terrain',
            'open_meadows',
            'obstacle_course',
            'modified',  # since you have this
        ],
    )
    # let you override nav2 params (default = drone)
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([config_path, 'nav2_drone.yaml']),
        description='Nav2 params file to use',
    )

    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(nav2_arg)
    ld.add_action(world_arg)
    ld.add_action(nav2_params_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    # --------------------
    # robot_description (drone)
    # --------------------
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher_node)

    # --------------------
    # robot_localization
    # --------------------
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )
    ld.add_action(robot_localization_node)

    # --------------------
    # Gazebo / Ignition
    # --------------------
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'),
            'launch',
            'ign_gazebo.launch.py'
        ]),
        launch_arguments={
            # this is the correct way to build <pkg>/worlds/<world>.sdf
            'ign_args': [
                PathJoinSubstitution([pkg_path, 'worlds', world + '.sdf']),
                ' -r'
            ]
        }.items()
    )
    ld.add_action(gazebo)

    # --------------------
    # spawn drone
    # --------------------
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/robot_description',
            '-name', 'drone',
            '-z', '2.0'
        ]
    )
    ld.add_action(robot_spawner)

    # --------------------
    # bridge
    # --------------------
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([
                config_path,
                'gazebo_bridge.yaml'
            ]),
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(gazebo_bridge)

    # --------------------
    # rviz (optional)
    # --------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # --------------------
    # nav2 (optional)
    # --------------------
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([
            pkg_path,
            'launch',
            '41068_navigation.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
