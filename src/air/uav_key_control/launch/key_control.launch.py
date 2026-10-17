from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_key_control',
            executable='keyboard_uav_control',
            output='screen',
            emulate_tty=True,
        )
    ])
