from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_linear_x', default_value='0.3'),
        DeclareLaunchArgument('max_angular_z', default_value='0.8'),
        DeclareLaunchArgument('cmd_timeout', default_value='2.0'),
        Node(
            package='cmd_vel_guard',
            executable='cmd_vel_guard',
            name='cmd_vel_guard',
            output='screen',
            parameters=[{
                'max_linear_x': LaunchConfiguration('max_linear_x'),
                'max_angular_z': LaunchConfiguration('max_angular_z'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
            }],
        ),
    ])
