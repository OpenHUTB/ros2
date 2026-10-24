"""
任务 4 Launch 文件 — 端到端视觉伺服管线巡检
=============================================
使用方式:
    ros2 launch rov_mujoco task4.launch.py

启动特性:
    1. mujoco_sim_node   - MuJoCo 物理仿真、前视相机 (/rov/camera/image_raw) 与里程计
    2. end_to_end_node   - 纯图像输入 -> /cmd_vel 的神经网络视觉伺服控制器

可用参数:
    architecture: mlp / small_cnn / nature_cnn (默认 nature_cnn)
    algorithm:    bc (行为克隆；当前发布的权值均由 BC 训练)
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
        DeclareLaunchArgument(
            'model_path', default_value=default_model,
            description='MuJoCo XML 模型文件路径'),
        DeclareLaunchArgument(
            'config_path', default_value=default_config,
            description='仿真配置文件路径'),
        DeclareLaunchArgument(
            'architecture', default_value='nature_cnn',
            description='视觉策略架构 (mlp / small_cnn / nature_cnn)'),
        DeclareLaunchArgument(
            'algorithm', default_value='bc',
            description='训练算法 (当前发布权值为 bc)'),
        DeclareLaunchArgument(
            'start_pose', default_value='-3.5,3.2,-1.45,0.0',
            description='ROV 初始位姿 x,y,z,yaw (管线巡检起点)'),

        # ========== 1. MuJoCo 物理仿真 + 传感器 ==========
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
                'enable_camera_render': True,
                # 端到端策略训练使用 84x84 原始渲染图，发布同样规格的图像，
                # 避免分辨率/水下色偏造成训练-部署分布不一致。
                'camera_width': 84,
                'camera_height': 84,
                'camera_underwater_effect': False,
                'start_pose': LaunchConfiguration('start_pose'),
            }]
        ),

        # ========== 2. 端到端视觉伺服节点 ==========
        Node(
            package='rov_mujoco',
            executable='end_to_end_node',
            name='end_to_end_node',
            output='screen',
            parameters=[{
                'architecture': LaunchConfiguration('architecture'),
                'algorithm': LaunchConfiguration('algorithm'),
                'nominal_speed': 0.45,
                'control_rate': 10.0,
                'goal_tolerance': 0.40,
            }]
        ),
    ])
