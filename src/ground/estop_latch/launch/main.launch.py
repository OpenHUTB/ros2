from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_timeout', default_value='0.0'),
        DeclareLaunchArgument(
            'release_hold_time', default_value='2.0'),
        DeclareLaunchArgument(
            'status_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'auto_release', default_value='false'),
        DeclareLaunchArgument(
            'start_latched', default_value='true'),
        Node(
            package='estop_latch',
            executable='estop_latch',
            name='estop_latch',
            output='screen',
            parameters=[{
                'input_timeout':
                    LaunchConfiguration('input_timeout'),
                'release_hold_time':
                    LaunchConfiguration('release_hold_time'),
                'status_rate': LaunchConfiguration('status_rate'),
                'auto_release': LaunchConfiguration('auto_release'),
                'start_latched': LaunchConfiguration('start_latched'),
            }],
        ),
    ])
