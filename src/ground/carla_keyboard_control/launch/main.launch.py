"""CARLA 地面载具键盘运动控制 —— ROS 2 Humble Launch 文件。

使用方式：
    ros2 launch carla_keyboard_control main.launch.py
    ros2 launch carla_keyboard_control main.launch.py host:=192.168.8.1 town:=Town05

启动节点：
    1. carla_control_node   —— 连接 CARLA、生成自车、驱动同步步进、广播传感器
    2. keyboard_teleop_node —— 读取键盘、发布控制指令
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('carla_keyboard_control')
    default_config = os.path.join(pkg_share, 'config', 'sim_params.yaml')

    return LaunchDescription([
        # ---------------- 参数声明 ----------------
        DeclareLaunchArgument(
            'host', default_value='127.0.0.1',
            description='CARLA 服务端地址（虚拟机运行客户端时填宿主机 IP）'),
        DeclareLaunchArgument(
            'port', default_value='2000', description='CARLA RPC 端口'),
        DeclareLaunchArgument(
            'town', default_value='Town05', description='CARLA 地图名'),
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='仿真参数 YAML 路径'),

        # ---------------- 仿真节点 ----------------
        Node(
            package='carla_keyboard_control',
            executable='carla_control_node',
            name='carla_control_node',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'town': LaunchConfiguration('town'),
            }],
        ),

        # ---------------- 键盘遥控节点 ----------------
        Node(
            package='carla_keyboard_control',
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('config')],
        ),
    ])
