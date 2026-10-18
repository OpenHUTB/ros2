"""main.launch.py —— 同时启动仿真节点与预测节点。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="humanoid_vision_fall_prediction",
            executable="vision_sim_node",
            name="vision_sim_node",
            output="screen",
        ),
        Node(
            package="humanoid_vision_fall_prediction",
            executable="fall_predictor_node",
            name="fall_predictor_node",
            output="screen",
        ),
    ])
