from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_linear_x', default_value='0.3'),
        DeclareLaunchArgument('max_linear_y', default_value='0.0'),
        DeclareLaunchArgument('max_angular_z', default_value='0.8'),
        DeclareLaunchArgument('cmd_timeout', default_value='2.0'),
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        Node(
            package='cmd_vel_guard',
            executable='cmd_vel_guard',
            name='cmd_vel_guard',
            output='screen',
            parameters=[{
                'max_linear_x': LaunchConfiguration('max_linear_x'),
                'max_linear_y': LaunchConfiguration('max_linear_y'),
                'max_angular_z': LaunchConfiguration('max_angular_z'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }],
        ),
    ])
