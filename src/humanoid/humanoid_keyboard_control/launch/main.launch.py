"""
人形机器人 MuJoCo 仿真与键盘遥控 Launch 文件
==========================================
使用方式:
    ros2 launch humanoid_keyboard_control main.launch.py

启动节点:
    1. mujoco_sim_node        - MuJoCo 仿真引擎（订阅 /cmd_vel，发布 /odom 等）
    2. keyboard_teleop_node   - 键盘遥控（发布 /cmd_vel）
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_keyboard_control')

    # 默认模型路径（安装后位于 share/humanoid_keyboard_control/models/）
    default_model = os.path.join(pkg_share, 'models', 'humanoid_simple.xml')

    return LaunchDescription([
        # ========== 参数声明 ==========
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model,
            description='人形机器人 MJCF 模型文件路径'
        ),
        DeclareLaunchArgument(
            'sim_rate',
            default_value='500.0',
            description='物理仿真频率 (Hz)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30.0',
            description='话题发布频率 (Hz)'
        ),
        DeclareLaunchArgument(
            'linear_scale',
            default_value='0.3',
            description='键盘线速度比例 (m/s)'
        ),
        DeclareLaunchArgument(
            'angular_scale',
            default_value='0.4',
            description='键盘角速度比例 (rad/s)'
        ),
        DeclareLaunchArgument(
            'cmd_timeout',
            default_value='0.5',
            description='指令超时停车时间 (s)'
        ),
        DeclareLaunchArgument(
            'visualize',
            default_value='true',
            description='是否打开 MuJoCo 可视化窗口（无图形界面时用 visualize:=false 关闭）'
        ),

        # ========== 仿真节点 ==========
        Node(
            package='humanoid_keyboard_control',
            executable='mujoco_sim_node',
            name='mujoco_sim_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'sim_rate': LaunchConfiguration('sim_rate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
                'visualize': LaunchConfiguration('visualize'),
            }]
        ),

        # ========== 键盘遥控节点 ==========
        Node(
            package='humanoid_keyboard_control',
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
            parameters=[{
                'linear_scale': LaunchConfiguration('linear_scale'),
                'angular_scale': LaunchConfiguration('angular_scale'),
            }]
        ),
    ])
