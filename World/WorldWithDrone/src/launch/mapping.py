from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Paths
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))

    # Robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_state_publisher)

    # Robot localization (odom + imu → fused /odom)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[PathJoinSubstitution([config_path, 'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(ekf_node)

    # Gazebo world
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py']),
        launch_arguments={'ign_args': [PathJoinSubstitution([pkg_path, 'worlds', 'large_demo.sdf']), ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Spawn Parrot
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'parrot', '-z', '2.0'],
        output='screen'
    )
    ld.add_action(spawn_robot)

    # Gazebo bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
                     'use_sim_time': use_sim_time}]
    )
    ld.add_action(bridge)

    # Slam Toolbox (online async mapping)
    slam_toolbox = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    ld.add_action(slam_toolbox)

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(rviz)

    return ld
