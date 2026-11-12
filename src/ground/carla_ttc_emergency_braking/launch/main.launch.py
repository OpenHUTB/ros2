"""ROS 2 launch wrapper for carla_ttc_emergency_braking.

用法：
    ros2 launch carla_ttc_emergency_braking main.launch.py host:=127.0.0.1 port:=2000 map:=Town10HD_Opt

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
                              description='仿真器所在主机（虚拟机里填 Windows VMnet8 地址）'),
        DeclareLaunchArgument('port', default_value='2000',
                              description='CARLA RPC 端口'),
        DeclareLaunchArgument('map', default_value='Town10HD_Opt',
                              description='地图名；留空沿用服务端当前地图'),
        Node(
            package='carla_ttc_emergency_braking',
            executable='carla_ttc_emergency_braking',   # console_scripts 里的入口，指向 carla_ttc_emergency_braking.app:main
            name='carla_ttc_emergency_braking',
            output='screen',
            arguments=['--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port'), '--map', LaunchConfiguration('map')],
        ),
    ])
