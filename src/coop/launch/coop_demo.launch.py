from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='coop',
            executable='coverage_manager',  # when we add it to console_scripts
            name='coverage_manager',
            output='screen'
        )
    ])
