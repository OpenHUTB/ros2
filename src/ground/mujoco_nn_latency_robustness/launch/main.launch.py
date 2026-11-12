"""launch 入口：按 config/params.yaml 起一个带执行延迟的跟踪节点。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "mujoco_nn_latency_robustness"


def _default_params() -> str:
    share = get_package_share_directory(PKG)
    return os.path.join(share, "config", "params.yaml")


def generate_launch_description() -> LaunchDescription:
    params = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=_default_params(),
                              description="节点参数文件"),
        DeclareLaunchArgument("delay_ms", default_value="600.0",
                              description="执行延迟（毫秒）"),
        DeclareLaunchArgument("comp", default_value="model",
                              description="补偿方式：none / extrap / model"),
        Node(
            package=PKG,
            executable="latency_nav_node",
            name="latency_nav_node",
            output="screen",
            parameters=[params, {"delay_ms": LaunchConfiguration("delay_ms"),
                                 "comp": LaunchConfiguration("comp")}],
        ),
    ])
