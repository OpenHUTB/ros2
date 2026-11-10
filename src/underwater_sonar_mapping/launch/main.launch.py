from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='underwater_sonar_mapping',
            executable='main',
            name='underwater_sonar_mapping_node',
            output='screen'
        )
    ])
