from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros2_opencv_tracking',
            executable='tracking_node',
            name='px4_opencv_tracking',
            output='screen',
        ),
    ])
