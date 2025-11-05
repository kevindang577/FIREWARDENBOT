#!/usr/bin/env python3
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    RegisterEventHandler,
    OpaqueFunction,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _find_world_file(sim_share: str, arg_value: str) -> str:
    """Resolve an absolute path to the world file."""
    if os.path.isabs(arg_value) and os.path.exists(arg_value):
        return arg_value
    if os.path.exists(arg_value):
        return os.path.abspath(arg_value)
    return os.path.join(sim_share, 'worlds', arg_value)


def _parse_world_name(world_file: str) -> str | None:
    """Parse <world name="..."> from SDF (first occurrence)."""
    try:
        with open(world_file, 'r', encoding='utf-8', errors='ignore') as f:
            head = f.read(8192)
        m = re.search(r'<\s*world[^>]*\sname\s*=\s*["\']([^"\']+)["\']', head)
        return m.group(1) if m else None
    except Exception:
        return None


def _build_actions(context):
    # ---- resolve args ----
    world_arg = LaunchConfiguration('world').perform(context)      # e.g., 'box_arena.sdf'
    drone_name = LaunchConfiguration('drone_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    # ---- package shares ----
    sim_share = get_package_share_directory('sim')
    bringup_share = get_package_share_directory('bringup')
    description_share = get_package_share_directory('description')

    # ---- resolve world path + internal name ----
    world_path = _find_world_file(sim_share, world_arg)
    if not os.path.exists(world_path):
        print(f"[sim_one_drone] ERROR: World file not found: {world_path}")
        print(f"[sim_one_drone]        Check it exists or is installed to {sim_share}/worlds/")
    else:
        print(f"[sim_one_drone] Using world file: {world_path}")

    world_name = _parse_world_name(world_path) if os.path.exists(world_path) else None
    if not world_name:
        world_name, _ = os.path.splitext(os.path.basename(world_path))
    print(f"[sim_one_drone] Internal world name: {world_name}")

    # ---- paths ----
    parrot_xacro = os.path.join(description_share, 'urdf', 'parrot.urdf.xacro')
    rviz_config = os.path.join(bringup_share, 'config', 'firewardenbot.rviz')
    tmp_urdf = '/tmp/parrot.urdf'

    # ---- ensure no old Gazebo servers are running (then wait, then start new) ----
    kill_gz_ign = ExecuteProcess(
        cmd=['/bin/bash', '-lc',
             "pkill -f 'ign[[:space:]]+gazebo' || true; "
             "pkill -f 'gz[[:space:]]+sim' || true"],
        output='screen'
    )

    # prefer `gz sim`, fallback to `ign gazebo`
    start_gazebo = ExecuteProcess(
        cmd=['/bin/bash', '-lc',
             f"(command -v gz >/dev/null 2>&1 && gz sim -r '{world_path}') "
             f"|| ign gazebo -r '{world_path}'"],
        output='screen'
    )

    # ✅ FIX: call start_gazebo after a short delay (must pass actions=[...])
    wait_after_kill = TimerAction(period=0.8, actions=[start_gazebo])

    # ---- resource paths (models + worlds) ----
    ign_paths = os.path.join(sim_share, 'models') + ':' + os.path.join(sim_share, 'worlds')
    set_ign_resources = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_paths)
    set_gz_resources  = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ign_paths)

    # ---- generate URDF from xacro (named with drone_name) ----
    gen_urdf = ExecuteProcess(
        cmd=['xacro', parrot_xacro, f'model_name:={drone_name}', '-o', tmp_urdf],
        output='screen'
    )

    # ---- robot_state_publisher reads the generated file ----
    robot_description = ParameterValue(Command(['cat ', tmp_urdf]), value_type=str)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    # ---- spawn explicitly into the parsed world name; forbid auto-renaming ----
    spawn_drone = ExecuteProcess(
        cmd=[
            '/bin/bash', '-lc',
            f"(ros2 run ros_gz_sim create -world '{world_name}' -name '{drone_name}' "
            f"-allow_renaming false -x 0 -y 0 -z 2.0 -file '{tmp_urdf}') "
            f"|| (ros2 run ros_ign_gazebo create -world '{world_name}' -name '{drone_name}' "
            f"-allow_renaming false -x 0 -y 0 -z 2.0 -file '{tmp_urdf}')"
        ],
        output='screen'
    )

    # start RSP + spawn only after URDF exists
    start_after_urdf = RegisterEventHandler(
        OnProcessExit(target_action=gen_urdf, on_exit=[rsp_node, spawn_drone])
    )

    # ---- bridge ----
    def _bridge_arg(suffix, ros_msg, gz_msg):
        return f"/model/{drone_name}{suffix}@{ros_msg}@{gz_msg}"

    bridge_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        arguments=[
            _bridge_arg('/cmd_vel', 'geometry_msgs/msg/Twist', 'ignition.msgs.Twist'),
            _bridge_arg('/odometry', 'nav_msgs/msg/Odometry', 'ignition.msgs.Odometry'),
            _bridge_arg('/imu', 'sensor_msgs/msg/Imu', 'ignition.msgs.IMU'),
            _bridge_arg('/scan', 'sensor_msgs/msg/LaserScan', 'ignition.msgs.LaserScan'),
            _bridge_arg('/camera', 'sensor_msgs/msg/Image', 'ignition.msgs.Image'),
        ]
    )

    # ---- rviz ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return [
        set_ign_resources,
        set_gz_resources,
        kill_gz_ign,       # kill any leftover servers
        wait_after_kill,   # then after 0.8s -> start Gazebo
        gen_urdf,          # build URDF
        start_after_urdf,  # then RSP + spawn
        bridge_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='flat_world_green.sdf',
                              description='World file (filename or absolute path)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('drone_name', default_value='drone1'),
        OpaqueFunction(function=_build_actions),
    ])
