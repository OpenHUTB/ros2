"""main.launch.py"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="humanoid_fall_warning", executable="warning_sim_node",
             name="warning_sim_node", output="screen"),
        Node(package="humanoid_fall_warning", executable="warning_predictor_node",
             name="warning_predictor_node", output="screen"),
    ])
