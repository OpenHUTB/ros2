"""
人形机器人平衡与抗扰恢复 Launch 文件
==================================
使用方式::

    ros2 launch humanoid_push_recovery main.launch.py

启动节点:
    1. balance_sim_node    - 平衡仿真（站姿 PD + 踝/髋力矩策略，发布 /balance/* 话题）
    2. push_teleop_node    - 扰动遥控（键盘发布 /balance/push）

参数来源：本 launch 把 config/params.yaml 作为**参数文件**传给两个节点，
因此 standalone 与 ROS2 使用同一套参数（params.yaml 按节点分节，
`/balance_sim_node` 段给仿真节点、`/push_teleop_node` 段给遥控节点）。
下面声明的 launch 参数作为**覆盖项**，优先级高于参数文件中的同名值。

常用覆盖示例::

    # 无图形界面（不弹可视化窗口）
    ros2 launch humanoid_push_recovery main.launch.py visualize:=false
    # 仅踝策略（关闭髋策略，做对照实验）
    ros2 launch humanoid_push_recovery main.launch.py hip_enable:=false
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_push_recovery')
    default_model = os.path.join(pkg_share, 'models', 'humanoid_balance.xml')
    # 由 setup.py 的 data_files 安装到 share/<pkg>/config/params.yaml
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # ========== 参数声明（覆盖 params.yaml 中的同名项）==========
        DeclareLaunchArgument('model_path', default_value=default_model,
                              description='人形机器人 MJCF 模型文件路径'),
        DeclareLaunchArgument('params_file', default_value=params_file,
                              description='参数文件路径（默认使用包内 config/params.yaml）'),
        DeclareLaunchArgument('sim_rate', default_value='500.0',
                              description='物理仿真频率 (Hz)'),
        DeclareLaunchArgument('publish_rate', default_value='30.0',
                              description='话题发布频率 (Hz)'),
        DeclareLaunchArgument('visualize', default_value='true',
                              description='是否打开 MuJoCo 可视化窗口（无图形界面时用 visualize:=false）'),
        DeclareLaunchArgument('hip_enable', default_value='true',
                              description='是否启用髋策略（false = 仅踝策略，用于对照实验）'),
        DeclareLaunchArgument('push_duration', default_value='0.08',
                              description='扰动持续时间 (s)'),
        DeclareLaunchArgument('force', default_value='120.0',
                              description='扰动遥控节点初始推扰力度 (N)'),

        # ========== 平衡仿真节点 ==========
        # parameters 列表中后者优先：参数文件先应用，launch 参数作为覆盖项
        Node(
            package='humanoid_push_recovery',
            executable='balance_sim_node',
            name='balance_sim_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'model_path': LaunchConfiguration('model_path'),
                'sim_rate': LaunchConfiguration('sim_rate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'visualize': LaunchConfiguration('visualize'),
                'hip_enable': LaunchConfiguration('hip_enable'),
                'push_duration': LaunchConfiguration('push_duration'),
            }]
        ),

        # ========== 扰动遥控节点 ==========
        Node(
            package='humanoid_push_recovery',
            executable='push_teleop_node',
            name='push_teleop_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'force': LaunchConfiguration('force')}]
        ),
    ])
