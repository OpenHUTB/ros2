"""main.launch.py"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="humanoid_temporal_e2e", executable="temporal_sim_node",
             name="temporal_sim_node", output="screen"),
        Node(package="humanoid_temporal_e2e", executable="temporal_predictor_node",
             name="temporal_predictor_node", output="screen"),
    ])
