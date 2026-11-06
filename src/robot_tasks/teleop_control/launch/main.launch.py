from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_control',
            executable='main_teleop',
            name='teleop_node',
            output='screen',
            emulate_tty=True
        )
    ])
