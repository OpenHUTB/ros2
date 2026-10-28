from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_recorder',
            executable='main',
            name='image_recorder_node',
            output='screen'
        )
    ])
