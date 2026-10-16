from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_key_control',
            executable='main_key_control.py',
            output='screen',
        )
    ])
