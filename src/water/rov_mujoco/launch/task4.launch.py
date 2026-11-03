"""
Launch 文件 — 基于深度神经网络的水下机器人端到端视觉控制模型
=============================================================
使用方式:
    ros2 launch rov_mujoco task4.launch.py
    ros2 launch rov_mujoco task4.launch.py architecture:=spatial_softmax algorithm:=sac

启动特性:
    1. mujoco_sim_node - 水下仿真物理引擎与相机图像广播 (/rov/camera/image_raw)
    2. end_to_end_vision_control_node - 纯视觉端到端神经网络前向推断，直接输出 /cmd_vel 控制指令
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rov_mujoco')

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
            'architecture',
            default_value='spatial_softmax',
            description='神经网络架构 (nature_cnn, resnet, spatial_softmax)'
        ),
        DeclareLaunchArgument(
            'algorithm',
            default_value='sac',
            description='策略强化学习/学习算法 (sac, ppo, bc)'
        ),
        DeclareLaunchArgument(
            'nominal_speed',
            default_value='0.45',
            description='水下巡航标称前进线速度 (m/s)'
        ),

        # ========== 1. MuJoCo 物理仿真节点 ==========
        Node(
            package='rov_mujoco',
            executable='mujoco_sim_node',
            name='mujoco_sim_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'config_path': LaunchConfiguration('config_path'),
                'sim_rate': 500.0,
                'publish_rate': 50.0,
                'use_ocean_current': True,
                'auto_trajectory_mode': False,
            }]
        ),

        # ========== 2. 端到端视觉控制节点 ==========
        Node(
            package='rov_mujoco',
            executable='end_to_end_vision_control_node',
            name='end_to_end_vision_control_node',
            output='screen',
            parameters=[{
                'architecture': LaunchConfiguration('architecture'),
                'algorithm': LaunchConfiguration('algorithm'),
                'nominal_speed': LaunchConfiguration('nominal_speed'),
            }]
        ),
    ])
