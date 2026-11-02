"""main.launch.py —— 启动深度图姿态估计双节点"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="humanoid_depth_pose_control",
             executable="depth_sim_node",
             name="depth_sim_node", output="screen"),
        Node(package="humanoid_depth_pose_control",
             executable="depth_predictor_node",
             name="depth_predictor_node", output="screen"),
    ])
