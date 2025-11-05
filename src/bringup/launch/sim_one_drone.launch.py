#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ----- args -----
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='large_demo.sdf',
        description='World file to load from the sim package'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # we keep this so later we can param-ify the robot_state_publisher
    drone_name_arg = DeclareLaunchArgument(
        'drone_name',
        default_value='drone1',
        description='Name/model_name of the drone'
    )

    # ----- paths -----
    sim_share = get_package_share_directory('sim')
    bringup_share = get_package_share_directory('bringup')
    description_share = get_package_share_directory('description')

    parrot_xacro = os.path.join(description_share, 'urdf', 'parrot.urdf.xacro')
    tmp_urdf = '/tmp/parrot.urdf'
    rviz_config = os.path.join(bringup_share, 'config', 'firewardenbot.rviz')

    # make gazebo find our models/worlds
    ign_paths = os.path.join(sim_share, 'models') + ':' + os.path.join(sim_share, 'worlds')
    set_ign_resources = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_paths)
    set_gz_resources = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ign_paths)

    # ----- start gazebo -----
    gazebo_proc = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r',
            PathJoinSubstitution([sim_share, 'worlds', LaunchConfiguration('world')])
        ],
        output='screen'
    )

    # ----- generate URDF from xacro (HARD-CODED model_name:=drone1) -----
    # this is the bit that was failing before; we make it super explicit now
    gen_urdf = ExecuteProcess(
        cmd=[
            'xacro',
            parrot_xacro,
            'model_name:=drone1',
            '-o', tmp_urdf
        ],
        output='screen'
    )

    # ----- robot_state_publisher (still uses launch arg) -----
    robot_description = ParameterValue(
        Command([
            'xacro ',
            parrot_xacro,
            ' model_name:=', LaunchConfiguration('drone_name')
        ]),
        value_type=str
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description
        }]
    )

    # ----- spawn (wait 2s so /tmp/parrot.urdf exists) -----
    spawn_drone = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('drone_name'),
            '-x', '0', '-y', '0', '-z', '2.0',
            '-file', tmp_urdf
        ]
    )
    delayed_spawn = TimerAction(period=2.0, actions=[spawn_drone])

    # ----- bridge -----
    bridge_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        arguments=[
            '/model/drone1/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/drone1/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/drone1/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/model/drone1/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/model/drone1/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
        ]
    )

    # ----- rviz -----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        drone_name_arg,
        set_ign_resources,
        set_gz_resources,
        gazebo_proc,
        gen_urdf,
        rsp_node,
        delayed_spawn,
        bridge_node,
        rviz_node,
    ])