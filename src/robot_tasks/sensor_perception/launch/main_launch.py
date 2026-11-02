from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_perception',
            executable='main_sensor',
            name='mock_camera',
            output='screen',
            parameters=[{
                'width': 320,
                'height': 240,
                'fps': 10.0,
            }],
        ),
    ])
