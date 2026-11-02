from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fire_detect',
            executable='fire_detector_node',
            name='fire_detector_node',
            output='screen'
        )
    ])
