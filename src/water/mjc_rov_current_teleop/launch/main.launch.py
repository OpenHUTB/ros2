"""
main.launch.py —— 一键启动 ROV 仿真（sim 节点）。
为什么默认只起 sim：键盘/巡航节点需要交互终端，单独 ros2 run 起更清晰；
本 launch 提供"仿真本体+话题"作为可被其他节点连接的服务。
"""
import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("mjc_rov_current_teleop")
    cfg = os.path.join(pkg_share, "config", "rov_current_params.yaml")
    return LaunchDescription([
        # 必须在 import mujoco 前设环境变量
        SetEnvironmentVariable("MUJOCO_GL", "osmesa"),
        SetEnvironmentVariable("PYOPENGL_PLATFORM", "osmesa"),
        Node(
            package="mjc_rov_current_teleop",
            executable="sim_node",
            name="rov_sim",
            parameters=[cfg],
            output="screen",
        ),
    ])
