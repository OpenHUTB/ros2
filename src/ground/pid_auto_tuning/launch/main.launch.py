from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid_auto_tuning',
            executable='main_node',
            name='pid_auto_tuning',
            output='screen',
        ),
    ])
