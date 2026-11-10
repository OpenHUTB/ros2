"""
任务 3 Launch 文件 — 水下 SLAM 占据栅格建图与神经网络自主导航
=============================================================
使用方式:
    ros2 launch rov_mujoco task3.launch.py

启动特性:
    1. mujoco_sim_node - 水下仿真物理引擎与传感器广播 (500Hz / 50Hz)
    2. sonar_slam_node - 水下多波束声呐贝叶斯占据栅格建图 (/map) 与 TF 广播
    3. autonomous_navigation_node - 神经网络路径规划与多航点自主避障巡航
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
            'planner_type',
            default_value='neural',
            description='规划器类型 (neural 或 baseline)'
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

        # ========== 2. 水下声呐 SLAM 建图节点 ==========
        Node(
            package='rov_mujoco',
            executable='sonar_slam_node',
            name='sonar_slam_node',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                'width_m': 20.0,
                'height_m': 20.0,
                'publish_rate': 5.0,
            }]
        ),

        # ========== 3. 神经网络多航点自主导航执行节点 ==========
        Node(
            package='rov_mujoco',
            executable='autonomous_navigation_node',
            name='autonomous_navigation_node',
            output='screen',
            parameters=[{
                'tolerance_reach': 0.25,
                'max_linear_speed': 0.55,
                'planner_type': LaunchConfiguration('planner_type'),
            }]
        ),
    ])
