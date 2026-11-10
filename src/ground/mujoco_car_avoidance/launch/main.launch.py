# -*- coding: utf-8 -*-
"""一键启动 ROS 2 节点。

用法：
    ros2 launch mujoco_car_avoidance main.launch.py
    ros2 launch mujoco_car_avoidance main.launch.py scene:=/abs/path/car_road_scene.xml
    ros2 launch mujoco_car_avoidance main.launch.py params_file:=/abs/path/params.yaml

注意（评审意见 #195 提到过的坑）：所有 <arg> 都要真正透传到节点参数，
否则 launch 传参是"看起来能传、实际没生效"。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "mujoco_car_avoidance"


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(package_share, "config", "params.yaml")
    default_scene = os.path.join(package_share, "models", "car_road_scene.xml")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="ROS 参数文件路径（避障阈值与话题配置）",
    )
    declare_scene = DeclareLaunchArgument(
        "scene",
        default_value=default_scene,
        description="MuJoCo 场景 XML 路径",
    )

    node = Node(
        package=PACKAGE_NAME,
        executable="mujoco_car_avoidance_node",
        name=PACKAGE_NAME,
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            # 用 ParameterValue 包一层，保证 scene 以字符串类型下发
            {"scene": ParameterValue(LaunchConfiguration("scene"), value_type=str)},
        ],
    )

    return LaunchDescription([declare_params_file, declare_scene, node])
