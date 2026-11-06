"""部署推理启动：ros2 launch carla_rl_driving carla_rl_infer.launch.py

前置：CARLA 模拟器 + ros-bridge 已运行，相机图像已发布。
本 launch 拉起端到端推理节点，闭环"图像 -> 策略 -> 阿克曼控制"。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='',
            description='可选：传入 config/rl_driving.yaml 绝对路径覆盖参数'),
        DeclareLaunchArgument(
            'model_path',
            default_value='models/ppo_cnn_town01.zip',
            description='训练好的策略权重路径'),
        Node(
            package='carla_rl_driving',
            executable='infer_node',
            name='carla_rl_infer_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
            }],
        ),
    ])
