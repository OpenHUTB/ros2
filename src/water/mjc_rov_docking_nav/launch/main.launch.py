"""main.launch.py — 一键启动对接仿真。"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("MUJOCO_GL", "osmesa"),
        SetEnvironmentVariable("PYOPENGL_PLATFORM", "osmesa"),
        Node(package="mjc_rov_docking_nav", executable="sim_node", name="rov_dock_sim", output="screen"),
        Node(package="mjc_rov_docking_nav", executable="dock_node", name="rov_dock_node", output="screen"),
    ])
