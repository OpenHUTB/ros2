"""人形机器人跌倒预判与保护性倒地 Launch。

用法：
    ros2 launch humanoid_fall_protection main.launch.py
    ros2 launch humanoid_fall_protection main.launch.py strategy:=limp
    ros2 launch humanoid_fall_protection main.launch.py tilt_threshold_deg:=25.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'humanoid_fall_protection'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(share, 'config', 'params.yaml')
    default_model = os.path.join(share, 'models', 'humanoid_fall.xml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ROS 参数文件路径'),
        DeclareLaunchArgument('model_path', default_value=default_model,
                              description='MuJoCo 模型 XML 路径'),
        DeclareLaunchArgument('strategy', default_value='protect',
                              description='rigid / limp / protect'),
        DeclareLaunchArgument('tilt_threshold_deg', default_value='30.0',
                              description='跌倒判据的倾角阈值（度）'),
        DeclareLaunchArgument('push_force', default_value='300.0',
                              description='推扰力 (N)'),
        Node(
            package=PACKAGE_NAME,
            executable='fall_protection_node',
            name='fall_protection_node',   # 必须与 config/params.yaml 的节点键一致，否则参数不生效
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {'model_path': LaunchConfiguration('model_path')},
                {'strategy': LaunchConfiguration('strategy')},
                {'tilt_threshold_deg': LaunchConfiguration('tilt_threshold_deg')},
                {'push_force': LaunchConfiguration('push_force')},
            ],
        ),
    ])
