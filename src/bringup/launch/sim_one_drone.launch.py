#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
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
    """Build a parameter_bridge argument as a VALID Python expression string."""
    return PythonExpression([
        TextSubstitution(text="'/model/' + "),
        TextSubstitution(text="'"), drone_name_cfg, TextSubstitution(text="'"),
        TextSubstitution(text=" + '"), TextSubstitution(text=suffix),
        TextSubstitution(text="@"), TextSubstitution(text=ros_msg),
        TextSubstitution(text="@"), TextSubstitution(text=gz_msg),
        TextSubstitution(text="'")
    ])


def _make_reset_and_remove(context):
    """
    Build reset/remove actions using resolved launch args.
    Runs after Gazebo starts so services are available.
    """
    world_arg = LaunchConfiguration('world').perform(context)  # e.g., 'box_arena.sdf'
    world_base, _ = os.path.splitext(world_arg)               # -> 'box_arena'
    drone_name = LaunchConfiguration('drone_name').perform(context)

    reset_world = ExecuteProcess(
        cmd=[
            'ign', 'service',
            '-s', f'/world/{world_base}/control',
            '--reqtype', 'ignition.msgs.WorldControl',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000',
            '--req', 'reset:{all:true}',
        ],
        output='screen'
    )

    remove_model = ExecuteProcess(
        cmd=[
            'ign', 'service',
            '-s', f'/world/{world_base}/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'name: "{drone_name}" type: MODEL',
        ],
        output='screen'
    )

    # Save values we’ll reuse later in the launch (world_base, drone_name)
    context.launch_configurations['__world_base__'] = world_base
    context.launch_configurations['__drone_name__'] = drone_name

    return [reset_world, remove_model]


def generate_launch_description():
    # ----- args -----
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='flat_world_green.sdf',
        description='World file to load from the sim package'
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    drone_name_arg   = DeclareLaunchArgument('drone_name',   default_value='drone1')

    # ----- package shares -----
    sim_share = get_package_share_directory('sim')
    bringup_share = get_package_share_directory('bringup')
    description_share = get_package_share_directory('description')

    parrot_xacro = os.path.join(description_share, 'urdf', 'parrot.urdf.xacro')
    tmp_urdf = '/tmp/parrot.urdf'
    rviz_config = os.path.join(bringup_share, 'config', 'firewardenbot.rviz')

    # ----- resources -----
    ign_paths = os.path.join(sim_share, 'models') + ':' + os.path.join(sim_share, 'worlds')
    set_ign_resources = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_paths)
    set_gz_resources  = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ign_paths)

    # ----- start Gazebo -----
    gazebo_proc = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r',
            PathJoinSubstitution([sim_share, 'worlds', LaunchConfiguration('world')])
        ],
        output='screen'
    )

    # ----- t≈1.5s: reset + remove (before spawn) -----
    clean_start = TimerAction(
        period=1.5,
        actions=[OpaqueFunction(function=_make_reset_and_remove)]
    )

    # ----- t≈3.0s: generate URDF from xacro (after cleanup) -----
    gen_urdf = ExecuteProcess(
        cmd=[
            'xacro',
            parrot_xacro,
            PythonExpression([
                TextSubstitution(text="'model_name:=' + '"),
                LaunchConfiguration('drone_name'),
                TextSubstitution(text="'")
            ]),
            '-o', tmp_urdf
        ],
        output='screen'
    )
    gen_urdf_after_cleanup = TimerAction(period=3.0, actions=[gen_urdf])

    # ----- robot_state_publisher -----
    robot_description = ParameterValue(Command(['cat ', tmp_urdf]), value_type=str)
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

    # ----- spawn (after URDF exists) -----
    # Use explicit world and forbid renaming so we always get exactly <drone_name>
    # If the name is taken, the create command will fail (which is good for debugging).
    spawn_drone = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_gazebo', 'create',
            '-world', PythonExpression(["'", LaunchConfiguration('__world_base__'), "'"]),
            '-name', LaunchConfiguration('drone_name'),
            '-allow_renaming', 'false',
            '-x', '0', '-y', '0', '-z', '2.0',
            '-file', tmp_urdf
        ],
        output='screen'
    )

    start_after_urdf = RegisterEventHandler(
        OnProcessExit(target_action=gen_urdf, on_exit=[rsp_node, spawn_drone])
    )

    # ----- bridge -----
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
        clean_start,              # cleanup (reset/remove)
        gen_urdf_after_cleanup,   # generate URDF
        start_after_urdf,         # then RSP + spawn
        bridge_node,
        rviz_node,
    ])
