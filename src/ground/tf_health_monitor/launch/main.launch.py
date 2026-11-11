from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stale_timeout', default_value='0.5'),
        DeclareLaunchArgument('rate_window', default_value='2.0'),
        DeclareLaunchArgument('min_rate', default_value='1.0'),
        DeclareLaunchArgument('min_samples', default_value='5'),
        DeclareLaunchArgument('backwards_hold', default_value='2.0'),
        DeclareLaunchArgument('stamp_lag', default_value='1.0'),
        DeclareLaunchArgument('publish_rate', default_value='5.0'),
        DeclareLaunchArgument('watched', default_value=''),
        Node(
            package='tf_health_monitor',
            executable='tf_health_monitor',
            name='tf_health_monitor',
            output='screen',
            parameters=[{
                'stale_timeout': LaunchConfiguration('stale_timeout'),
                'rate_window': LaunchConfiguration('rate_window'),
                'min_rate': LaunchConfiguration('min_rate'),
                'min_samples': LaunchConfiguration('min_samples'),
                'backwards_hold': LaunchConfiguration('backwards_hold'),
                'stamp_lag': LaunchConfiguration('stamp_lag'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'watched': LaunchConfiguration('watched'),
            }],
        ),
    ])
