"""humanoid_end_to_end 一键启动。

    ros2 launch humanoid_end_to_end main.launch.py
    ros2 launch humanoid_end_to_end main.launch.py teleop:=false
    ros2 launch humanoid_end_to_end main.launch.py model_path:=/abs/path/scene_e2e.xml
    ros2 launch humanoid_end_to_end main.launch.py weights:=/abs/path/e2e_policy.pt

启动内容：
    1. sim_node         MuJoCo 物理仿真 + 摄像头渲染（发布 image / odom / obs）
    2. policy_node      端到端 CNN 策略推理（image + obs -> joint_targets）
    3. keyboard_teleop  键盘遥控（-> /cmd_vel）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("humanoid_end_to_end")

    default_model = os.path.join(share, "models", "scene_e2e.xml")
    default_weights = os.path.join(share, "weights", "e2e_policy.pt")

    model_path = LaunchConfiguration("model_path")
    weights = LaunchConfiguration("weights")
    teleop = LaunchConfiguration("teleop")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model,
                description="G1 端到端控制的 MuJoCo 场景文件路径",
            ),
            DeclareLaunchArgument(
                "weights",
                default_value=default_weights,
                description="端到端 CNN 策略权重路径",
            ),
            DeclareLaunchArgument(
                "teleop",
                default_value="true",
                description="是否启动键盘遥控节点（无终端时可设为 false）",
            ),
            Node(
                package="humanoid_end_to_end",
                executable="sim_node",
                name="humanoid_e2e_sim_node",
                output="screen",
                parameters=[
                    {"model_path": model_path},
                    {"sim_rate": 250.0},
                    {"image_rate": 10.0},
                    {"publish_tf": True},
                ],
            ),
            Node(
                package="humanoid_end_to_end",
                executable="policy_node",
                name="humanoid_e2e_policy_node",
                output="screen",
                parameters=[
                    {"weights": weights},
                    {"inference_rate": 10.0},
                    {"device": "cpu"},
                ],
            ),
            Node(
                package="humanoid_end_to_end",
                executable="keyboard_teleop",
                name="e2e_keyboard_teleop",
                output="screen",
                prefix="xterm -e",
                condition=IfCondition(teleop),
            ),
        ]
    )
