"""CARLA 建图 + 导航（神经网络规划）—— ROS 2 Humble Launch 文件。

使用方式：
    ros2 launch carla_mapping_navigation main.launch.py
    ros2 launch carla_mapping_navigation main.launch.py host:=192.168.8.1 goal:="20,8"

说明：
    建图用激光雷达占用栅格（对数几率贝叶斯更新），导航用规划神经网络。
    CARLA 服务端启动、宿主机 IP 与端口 2000 的查看等通用步骤见
    https://openhutb.github.io/ros2/set_up_and_connect_to_carla/ ，本模块不重复描述。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('carla_mapping_navigation')
    default_config = os.path.join(pkg_share, 'config', 'sim_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'host', default_value='127.0.0.1',
            description='CARLA 服务端地址（虚拟机运行客户端时填宿主机 IP）'),
        DeclareLaunchArgument('port', default_value='2000', description='CARLA RPC 端口'),
        DeclareLaunchArgument('town', default_value='Town05', description='CARLA 地图名'),
        DeclareLaunchArgument('goal_x', default_value='20.0', description='导航目标 x（m）'),
        DeclareLaunchArgument('goal_y', default_value='8.0', description='导航目标 y（m）'),
        DeclareLaunchArgument(
            'config', default_value=default_config, description='仿真参数 YAML 路径'),

        Node(
            package='carla_mapping_navigation',
            executable='mapping_navigation_node',
            name='carla_mapping_navigation_node',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'town': LaunchConfiguration('town'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
            }],
        ),
    ])
