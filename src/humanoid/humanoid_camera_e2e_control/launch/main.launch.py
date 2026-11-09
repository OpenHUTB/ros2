# -*- coding: utf-8 -*-
"""
人形机器人图像端到端控制 Launch 文件
====================================
使用方式:
    # 端到端推理（仿真节点 + CNN 策略节点）
    ros2 launch humanoid_camera_e2e_control main.launch.py mode:=run

    # 键盘驾驶采集（仿真节点 + 键盘遥控节点，配合 standalone 采集数据）
    ros2 launch humanoid_camera_e2e_control main.launch.py mode:=collect

    # 关闭可视化窗口（无图形环境）
    ros2 launch humanoid_camera_e2e_control main.launch.py visualize:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_camera_e2e_control')
    default_model = os.path.join(pkg_share, 'models', 'humanoid_obstacle.xml')
    default_weights = os.path.join(pkg_share, 'weights', 'humanoid_cnn.pt')
    mode = LaunchConfiguration('mode')
    mode_is_run = PythonExpression(["'", mode, "' == 'run'"])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='run',
                              description='run=CNN 端到端推理；collect=键盘驾驶采集'),
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('weights_path', default_value=default_weights),
        DeclareLaunchArgument('sim_rate', default_value='500.0'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('image_rate', default_value='10.0'),
        DeclareLaunchArgument('control_period', default_value='0.05'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument('visualize', default_value='true'),
        DeclareLaunchArgument('linear_scale', default_value='0.22'),
        DeclareLaunchArgument('angular_scale', default_value='0.35'),

        # 仿真节点（两种模式都启动）
        Node(
            package='humanoid_camera_e2e_control',
            executable='mujoco_sim_node',
            name='mujoco_sim_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'sim_rate': LaunchConfiguration('sim_rate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'image_rate': LaunchConfiguration('image_rate'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
                'visualize': LaunchConfiguration('visualize'),
            }]
        ),

        # CNN 端到端策略节点（mode:=run 启动）
        Node(
            package='humanoid_camera_e2e_control',
            executable='e2e_policy_node',
            name='e2e_policy_node',
            output='screen',
            condition=IfCondition(mode_is_run),
            parameters=[{
                'weights_path': LaunchConfiguration('weights_path'),
                'control_period': LaunchConfiguration('control_period'),
                'image_topic': '/humanoid/camera/image_raw',
                'cmd_topic': '/cmd_vel',
            }]
        ),

        # 键盘遥控节点（mode:=collect 启动）
        Node(
            package='humanoid_camera_e2e_control',
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
            condition=UnlessCondition(mode_is_run),
            parameters=[{
                'linear_scale': LaunchConfiguration('linear_scale'),
                'angular_scale': LaunchConfiguration('angular_scale'),
            }]
        ),
    ])
