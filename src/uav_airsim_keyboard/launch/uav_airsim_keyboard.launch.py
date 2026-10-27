from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_airsim_keyboard',
            executable='keyboard_node',
            name='uav_airsim_keyboard',
            output='screen',
        )
    ])
