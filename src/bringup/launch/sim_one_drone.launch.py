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
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bridge_arg(drone_name_cfg, suffix, ros_msg, gz_msg):
    """
    Build a parameter_bridge argument string using a VALID Python expression,
    e.g. '/model/' + '<name>' + '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
    """
    return PythonExpression([
        # begin quoted python string: '/model/' +
        TextSubstitution(text="'/model/' + "),
        # insert the drone_name as a quoted string: '<value>'
        TextSubstitution(text="'"), drone_name_cfg, TextSubstitution(text="'"),
        # + '<suffix>@<ros>@<gz>'
        TextSubstitution(text=" + '"), TextSubstitution(text=suffix),
        TextSubstitution(text="@"), TextSubstitution(text=ros_msg),
        TextSubstitution(text="@"), TextSubstitution(text=gz_msg),
        TextSubstitution(text="'")
    ])


def generate_launch_description():
    # ----- args -----
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='flat_world_green.sdf',
        description='World file to load from the sim package'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    drone_name_arg = DeclareLaunchArgument(
        'drone_name',
        default_value='drone1',
        description='Name/model_name of the drone'
    )

    # ----- package shares -----
    sim_share = get_package_share_directory('sim')
    bringup_share = get_package_share_directory('bringup')
    description_share = get_package_share_directory('description')

    # Ensure this xacro exists (see earlier step to align filename/location)
    parrot_xacro = os.path.join(description_share, 'urdf', 'parrot.urdf.xacro')
    tmp_urdf = '/tmp/parrot.urdf'
    rviz_config = os.path.join(bringup_share, 'config', 'firewardenbot.rviz')

    # ----- make gazebo find our models/worlds -----
    ign_paths = os.path.join(sim_share, 'models') + ':' + os.path.join(sim_share, 'worlds')
    set_ign_resources = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_paths)
    set_gz_resources = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ign_paths)

    # ----- start gazebo (world path under sim/worlds) -----
    gazebo_proc = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r',
            PathJoinSubstitution([sim_share, 'worlds', LaunchConfiguration('world')])
        ],
        output='screen'
    )

    # ----- generate URDF from xacro (VALID Python expression for the arg) -----
    gen_urdf = ExecuteProcess(
        cmd=[
            'xacro',
            parrot_xacro,
            # Build "model_name:=<drone_name>" as a proper Python string expression
            PythonExpression([
                TextSubstitution(text="'model_name:=' + '"),
                LaunchConfiguration('drone_name'),
                TextSubstitution(text="'")
            ]),
            '-o', tmp_urdf
        ],
        output='screen'
    )

    # ----- robot_state_publisher reads the generated file (avoid drift) -----
    robot_description = ParameterValue(
        Command(['cat ', tmp_urdf]),
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

    # ----- spawn (small delay to ensure /tmp/parrot.urdf exists) -----
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

    # ----- bridge (topics parameterized by drone_name) -----
    dn = LaunchConfiguration('drone_name')
    bridge_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        arguments=[
            _bridge_arg(dn, '/cmd_vel', 'geometry_msgs/msg/Twist', 'ignition.msgs.Twist'),
            _bridge_arg(dn, '/odometry', 'nav_msgs/msg/Odometry', 'ignition.msgs.Odometry'),
            _bridge_arg(dn, '/imu', 'sensor_msgs/msg/Imu', 'ignition.msgs.IMU'),
            _bridge_arg(dn, '/scan', 'sensor_msgs/msg/LaserScan', 'ignition.msgs.LaserScan'),
            _bridge_arg(dn, '/camera', 'sensor_msgs/msg/Image', 'ignition.msgs.Image'),
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