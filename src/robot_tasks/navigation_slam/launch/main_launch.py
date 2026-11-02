from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_slam',
            executable='main_map',
            name='map_publisher',
            output='screen',
        ),
        Node(
            package='navigation_slam',
            executable='main_planner',
            name='path_planner',
            output='screen',
            parameters=[{
                'start_x': 0.5,
                'start_y': 0.5,
            }],
        ),
    ])
