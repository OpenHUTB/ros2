from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='192.168.43.1',
                               description='CARLA server IP'),
        DeclareLaunchArgument('port', default_value='2000',
                               description='CARLA server port'),
        Node(
            package='carla_parking_lot',
            executable='main',
            name='parking_lot_node',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
            }],
        ),
    ])

