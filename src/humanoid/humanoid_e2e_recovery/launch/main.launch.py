"""main.launch.py —— 同时启动仿真节点与端到端推理节点。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="humanoid_e2e_recovery",
            executable="e2e_sim_node",
            name="e2e_sim_node",
            output="screen",
        ),
        Node(
            package="humanoid_e2e_recovery",
            executable="e2e_predictor_node",
            name="e2e_predictor_node",
            output="screen",
        ),
    ])
