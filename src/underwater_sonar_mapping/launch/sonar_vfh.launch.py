from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='underwater_sonar_mapping',
            executable='sonar_vfh',
            name='sonar_vfh',
            output='screen',
        )
    ])
