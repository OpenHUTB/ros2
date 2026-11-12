from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_lidar_obstacle',
            executable='obstacle_avoid',
            name='obstacle_avoid',
            output='screen',
        ),
    ])
