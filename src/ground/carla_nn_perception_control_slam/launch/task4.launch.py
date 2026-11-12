from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration('carla_host')
    port = LaunchConfiguration('carla_port')
    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='127.0.0.1',
                              description='CARLA 服务器地址'),
        DeclareLaunchArgument('carla_port', default_value='2000',
                              description='CARLA 服务器端口'),
        Node(
            package='carla_nn_perception_control_slam',
            executable='main',
            name='carla_task4',
            output='screen',
            arguments=['--task', '4', '--host', host, '--port', port,
        '--mode', 'drive', '--frames', '70',],
        ),
    ])
