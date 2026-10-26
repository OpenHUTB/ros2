"""main.launch.py — 一键启动 RL 对比仿真。"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("MUJOCO_GL", "osmesa"),
        SetEnvironmentVariable("PYOPENGL_PLATFORM", "osmesa"),
        Node(package="mjc_rov_rl_compare", executable="sim_node", name="rov_rl_sim", output="screen"),
    ])
