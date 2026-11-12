"""ROS 2 launch wrapper for airsim_multirotor_waypoint_nav.

用法：
    ros2 launch airsim_multirotor_waypoint_nav main.launch.py host:=127.0.0.1 port:=41451

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
                              description='AirSim RPC 主机'),
        DeclareLaunchArgument('port', default_value='41451',
                              description='AirSim RPC 端口'),
        Node(
            package='airsim_multirotor_waypoint_nav',
            executable='airsim_multirotor_waypoint_nav',   # console_scripts 里的入口，指向 airsim_multirotor_waypoint_nav.app:main
            name='airsim_multirotor_waypoint_nav',
            output='screen',
            arguments=['--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port')],
        ),
    ])
