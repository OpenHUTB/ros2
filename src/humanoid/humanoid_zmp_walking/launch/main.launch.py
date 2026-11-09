from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_zmp_walking',
            executable='zmp_generator',
            name='zmp_generator_node',
            output='screen'
        ),
        Node(
            package='humanoid_zmp_walking',
            executable='play',
            name='play_node',
            output='screen',
            parameters=[{'use_gui': True}]
        ),
    ])
