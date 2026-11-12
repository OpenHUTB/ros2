"""ROS 2 launch wrapper for carla_lidar_obstacle_detection.

用法：
    ros2 launch carla_lidar_obstacle_detection main.launch.py host:=127.0.0.1 port:=2000 map:=Town10HD_Opt

本模块通过 CARLA / AirSim / MuJoCo 的 Python API 直连仿真器（不启动 carla_ros_bridge），
launch 文件只负责参数声明与进程包装；可执行文件由 setup.py 的 console_scripts 注册。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1',
                              description='仿真器所在主机'),
        DeclareLaunchArgument('port', default_value='2000',
                              description='CARLA RPC 端口'),
        DeclareLaunchArgument('map', default_value='Town10HD_Opt',
                              description='地图名'),
        Node(
            package='carla_lidar_obstacle_detection',
            executable='carla_lidar_obstacle_detection',   # console_scripts 里的入口，指向 carla_lidar_obstacle_detection.app:main
            name='carla_lidar_obstacle_detection',
            output='screen',
            arguments=['--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port'), '--map', LaunchConfiguration('map')],
        ),
    ])
