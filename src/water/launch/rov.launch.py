from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='water',
            executable='sonar_infer_node',
            name='sonar_infer_node',
            output='screen'
        )
    ])
