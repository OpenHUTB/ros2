from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros2_opencv_tracking',
            executable='image_pub',
            name='image_pub',
            output='screen',
        ),
        Node(
            package='px4_ros2_opencv_tracking',
            executable='color_tracker',
            name='color_tracker',
            output='screen',
        ),
        Node(
            package='px4_ros2_opencv_tracking',
            executable='offboard_control',
            name='offboard_control',
            output='screen',
        ),
    ])
