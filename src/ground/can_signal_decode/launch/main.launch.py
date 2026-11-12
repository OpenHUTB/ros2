from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_signal_decode',
            executable='main_node',
            name='can_signal_decode',
            output='screen',
        ),
    ])
