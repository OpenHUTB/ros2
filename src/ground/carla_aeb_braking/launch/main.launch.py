"""AEB 一键启动：CARLA 桥接节点 + AEB 控制节点。

用法：
    ros2 launch carla_aeb_braking main.launch.py host:=<CARLA 所在机器 IP>

可选参数：
    host:=127.0.0.1      CARLA 服务器 IP（虚拟机跑 ROS、宿主机跑 CARLA 时填宿主机 IP）
    port:=2000           CARLA 端口
    town:=Town01_Opt     地图（服务端已加载时跳过重载）
    start_armed:=true    启动时 AEB 是否启用（false 时车辆仅巡航、不自动制动）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    town = LaunchConfiguration("town")
    start_armed = LaunchConfiguration("start_armed")
    params_yaml = os.path.join(
        get_package_share_directory("carla_aeb_braking"), "config", "aeb_params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1",
                              description="CARLA 服务器 IP"),
        DeclareLaunchArgument("port", default_value="2000"),
        DeclareLaunchArgument("town", default_value="Town01_Opt"),
        DeclareLaunchArgument("start_armed", default_value="true"),
        Node(
            package="carla_aeb_braking",
            executable="aeb_bridge_node",
            name="aeb_bridge_node",
            output="screen",
            parameters=[{"host": host, "port": port, "town": town}],
        ),
        Node(
            package="carla_aeb_braking",
            executable="aeb_controller_node",
            name="aeb_controller_node",
            output="screen",
            parameters=[params_yaml, {"start_armed": start_armed}],
        ),
    ])
