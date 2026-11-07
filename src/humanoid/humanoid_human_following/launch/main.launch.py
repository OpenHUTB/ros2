"""
humanoid_human_following Launch 文件
====================================
使用方式:
    ros2 launch humanoid_human_following main.launch.py
    ros2 launch humanoid_human_following main.launch.py visualize:=true   # 打开可视化窗口
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'control_hz', default_value='20.0',
            description='控制频率 (Hz)'),
        DeclareLaunchArgument(
            'd_keep', default_value='0.8',
            description='期望跟随距离 (m)'),
        DeclareLaunchArgument(
            'visualize', default_value='false',
            description='是否打开 MuJoCo 可视化窗口'),

        Node(
            package='humanoid_human_following',
            executable='mujoco_sim_node',
            name='humanoid_follow_node',
            output='screen',
            parameters=[{
                'control_hz': LaunchConfiguration('control_hz'),
                'd_keep': LaunchConfiguration('d_keep'),
                'visualize': LaunchConfiguration('visualize'),
            }]
        ),
    ])
