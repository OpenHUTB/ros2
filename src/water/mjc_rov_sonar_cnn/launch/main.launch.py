"""main.launch.py — 一键启动声呐仿真节点。"""
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("MUJOCO_GL", "osmesa"),
        SetEnvironmentVariable("PYOPENGL_PLATFORM", "osmesa"),
        Node(
            package="mjc_rov_sonar_cnn",
            executable="sim_node",
            name="rov_sonar_sim",
            output="screen",
        ),
    ])
