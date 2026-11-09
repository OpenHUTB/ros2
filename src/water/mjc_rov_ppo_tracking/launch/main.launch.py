"""main.launch.py — 一键启动 PPO 仿真。"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("MUJOCO_GL", "osmesa"),
        SetEnvironmentVariable("PYOPENGL_PLATFORM", "osmesa"),
        Node(package="mjc_rov_ppo_tracking", executable="sim_node", name="rov_ppo_sim", output="screen"),
        Node(package="mjc_rov_ppo_tracking", executable="ppo_node", name="rov_ppo_node", output="screen"),
    ])
