from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('uav_keyboard_control')
    return LaunchDescription([
        Node(
            package='uav_keyboard_control',
            executable='keyboard',
            name='uav_keyboard_control',
            output='screen',
            parameters=[os.path.join(share, 'config', 'keyboard.yaml')],
        ),
    ])
