from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('velocity_deadband', default_value='0.02'),
        DeclareLaunchArgument('stuck_ratio', default_value='0.1'),
        DeclareLaunchArgument('slip_ratio', default_value='1.5'),
        DeclareLaunchArgument('stuck_duration', default_value='0.5'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument('odom_timeout', default_value='0.5'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        Node(
            package='odom_health_monitor',
            executable='odom_health_monitor',
            name='odom_health_monitor',
            output='screen',
            parameters=[{
                'velocity_deadband':
                    LaunchConfiguration('velocity_deadband'),
                'stuck_ratio': LaunchConfiguration('stuck_ratio'),
                'slip_ratio': LaunchConfiguration('slip_ratio'),
                'stuck_duration': LaunchConfiguration('stuck_duration'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
                'odom_timeout': LaunchConfiguration('odom_timeout'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }],
        ),
    ])
