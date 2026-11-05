"""
humanoid_visual_servo_grasp Launch 文件
=======================================
使用方式:
    ros2 launch humanoid_visual_servo_grasp main.launch.py
    ros2 launch humanoid_visual_servo_grasp main.launch.py visualize:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'control_hz', default_value='50.0',
            description='控制频率 (Hz)'),
        DeclareLaunchArgument(
            'visualize', default_value='false',
            description='是否打开 MuJoCo 可视化窗口'),

        Node(
            package='humanoid_visual_servo_grasp',
            executable='mujoco_sim_node',
            name='visual_servo_grasp_node',
            output='screen',
            parameters=[{
                'control_hz': LaunchConfiguration('control_hz'),
                'visualize': LaunchConfiguration('visualize'),
            }]
        ),
    ])
