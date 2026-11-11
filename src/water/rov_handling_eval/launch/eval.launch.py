# -*- coding: utf-8 -*-
"""一键启动：物理内核 + 评测节点。

键盘节点**不放在 launch 里**：终端只在"前台"才读得到按键，
被 launch 托管后 stdin 不是终端，按键会失效。请另开终端：

    ros2 run rov_handling_eval keyboard_teleop

用法::

    ros2 launch rov_handling_eval eval.launch.py
    ros2 launch rov_handling_eval eval.launch.py current_speed:=0.3 ballast:=0.27
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("rov_handling_eval")
    params = os.path.join(share, "config", "handling.yaml")

    args = [
        DeclareLaunchArgument("current_speed", default_value="0.0",
                              description="平均流速 m/s（0.1/0.2/0.3/0.5 是实验档位）"),
        DeclareLaunchArgument("ballast", default_value="0.0",
                              description="额外配重 kg（-0.5 / 0 / 0.27 / 0.5 / 1.0 是实验档位）"),
        DeclareLaunchArgument("csv_path", default_value="",
                              description="非空则把评测指标写成 CSV"),
    ]

    overrides = {
        "mujoco_sim_node": {
            "current_speed": LaunchConfiguration("current_speed"),
            "ballast": LaunchConfiguration("ballast"),
        },
        "eval_node": {
            "ballast": LaunchConfiguration("ballast"),
            "csv_path": LaunchConfiguration("csv_path"),
        },
    }

    return LaunchDescription(args + [
        Node(package="rov_handling_eval", executable="mujoco_sim",
             name="mujoco_sim_node", output="screen",
             parameters=[params, overrides["mujoco_sim_node"]]),
        Node(package="rov_handling_eval", executable="eval_node",
             name="eval_node", output="screen",
             parameters=[params, overrides["eval_node"]]),
    ])
