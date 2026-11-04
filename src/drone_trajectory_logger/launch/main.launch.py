from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_trajectory_logger',
            executable='main',
            name='trajectory_publisher',
            output='screen'
        ),
        Node(
            package='drone_trajectory_logger',
            executable='analyzer',
            name='trajectory_analyzer',
            output='screen'
        )
    ])
