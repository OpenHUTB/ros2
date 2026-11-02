"""CSL 一键启动：CARLA 桥接节点 + 速度控制节点。

用法（host 填 CARLA 所在机器 IP）：
    ros2 launch carla_curve_speed_limit main.launch.py host:=192.168.1.100
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMS = os.path.join(get_package_share_directory("carla_curve_speed_limit"), "config", "csl_params.yaml")


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="2000"),
        Node(
            package="carla_curve_speed_limit",
            executable="csl_bridge_node",
            name="csl_bridge_node",
            output="screen",
            parameters=[PARAMS, {"host": host, "port": port}],
        ),
        Node(
            package="carla_curve_speed_limit",
            executable="csl_control_node",
            name="csl_control_node",
            output="screen",
            parameters=[PARAMS],
        ),
    ])
