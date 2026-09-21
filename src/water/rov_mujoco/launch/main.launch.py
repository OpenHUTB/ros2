"""
水下机器人 MuJoCo 仿真与键盘遥控 Launch 文件
==========================================
使用方式:
    ros2 launch rov_mujoco main.launch.py

启动节点:
    1. mujoco_sim_node - MuJoCo 仿真引擎
    2. keyboard_teleop_node - 键盘遥控
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rov_mujoco')

    # 默认路径
    default_model = os.path.join(pkg_share, 'models', 'underwater_rov_with_arm.xml')
    default_config = os.path.join(pkg_share, 'models', 'config.json')

    return LaunchDescription([
        # ========== 参数声明 ==========
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='MuJoCo XML 模型文件路径'
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config,
            description='仿真配置文件路径'
        ),
        DeclareLaunchArgument(
            'sim_rate',
            default_value='500.0',
            description='物理仿真频率 (Hz)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='话题发布频率 (Hz)'
        ),

        # ========== 仿真节点 ==========
        Node(
            package='rov_mujoco',
            executable='mujoco_sim_node',
            name='mujoco_sim_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'config_path': LaunchConfiguration('config_path'),
                'sim_rate': LaunchConfiguration('sim_rate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'use_ocean_current': True,
            }]
        ),

        # ========== 键盘遥控节点 ==========
        Node(
            package='rov_mujoco',
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
            parameters=[{
                'linear_scale': 1.0,
                'angular_scale': 1.0,
            }]
        ),
    ])
