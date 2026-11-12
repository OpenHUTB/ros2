from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'watched_nodes', default_value='navigator'),
        DeclareLaunchArgument(
            'heartbeat_timeout', default_value='1.0'),
        DeclareLaunchArgument(
            'publish_rate', default_value='10.0'),
        DeclareLaunchArgument(
            'publish_estop', default_value='true'),
        Node(
            package='heartbeat_watchdog',
            executable='heartbeat_watchdog',
            name='heartbeat_watchdog',
            output='screen',
            parameters=[{
                'watched_nodes': LaunchConfiguration('watched_nodes'),
                'heartbeat_timeout':
                    LaunchConfiguration('heartbeat_timeout'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'publish_estop': LaunchConfiguration('publish_estop'),
            }],
        ),
    ])
