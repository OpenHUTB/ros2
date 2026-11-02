"""BSD 一键启动：CARLA 桥接节点 + BSD 预警节点。

用法（host 填 CARLA 所在机器 IP）：
    ros2 launch carla_bsd_blind_spot main.launch.py host:=192.168.1.100
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMS = os.path.join(get_package_share_directory("carla_bsd_blind_spot"), "config", "bsd_params.yaml")


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="2000"),
        Node(
            package="carla_bsd_blind_spot",
            executable="bsd_bridge_node",
            name="bsd_bridge_node",
            output="screen",
            parameters=[PARAMS, {"host": host, "port": port}],
        ),
        Node(
            package="carla_bsd_blind_spot",
            executable="bsd_alert_node",
            name="bsd_alert_node",
            output="screen",
            parameters=[PARAMS],
        ),
    ])
