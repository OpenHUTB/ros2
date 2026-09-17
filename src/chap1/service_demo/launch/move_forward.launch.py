from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        Node(
            package='service_demo',
            executable='server',
            name='move_forward_server',
            output='screen'
        ),
    ])
