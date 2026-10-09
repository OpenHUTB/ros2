"""
水下多传感器感知与 3D 轨迹自主跟踪 Launch 文件
===============================================
使用方式:
    ros2 launch rov_mujoco trajectory_tracking.launch.py

启动特性:
    1. mujoco_sim_node - 自动启用 auto_trajectory_mode=True
    2. 多传感器广播 - /rov/sonar/scan, /rov/camera/image_raw, /rov/imu, /rov/desired_path
    3. 神经网络闭环跟踪控制器 (NN MLP Policy)
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
            'sim_rate',
            default_value='500.0',
            description='物理仿真频率 (Hz)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='话题发布频率 (Hz)'
        ),
        DeclareLaunchArgument(
            'trajectory_type',
            default_value='3d_helix',
            description='跟踪轨迹类型 (3d_helix 或 lawnmower)'
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value='nn',
            description='控制器类型 (nn 或 pid)'
        ),

        # ========== 仿真节点 (自包含神经网络跟踪与多传感器) ==========
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
                'auto_trajectory_mode': True,
                'trajectory_type': LaunchConfiguration('trajectory_type'),
                'controller_type': LaunchConfiguration('controller_type'),
            }]
        ),
    ])
