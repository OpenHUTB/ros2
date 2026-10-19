"""humanoid_ppo_locomotion 一键启动。

    ros2 launch humanoid_ppo_locomotion main.launch.py
    ros2 launch humanoid_ppo_locomotion main.launch.py teleop:=false
    ros2 launch humanoid_ppo_locomotion main.launch.py model_path:=/abs/path/scene_29dof.xml

启动内容：
    1. sim_node         MuJoCo 物理仿真 + 解析平衡基元（发布 joint_states / odom / obs）
    2. policy_node      PPO 残差策略推理（obs -> joint_targets）
    3. keyboard_teleop  键盘驱动上半身姿态（-> /humanoid/upper_body）与速度指令（-> /cmd_vel）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("humanoid_ppo_locomotion")

    default_model = os.path.join(share, "models", "g1", "scene_29dof.xml")
    default_weights = os.path.join(share, "weights", "policy_g1.pt")

    model_path = LaunchConfiguration("model_path")
    weights = LaunchConfiguration("weights")
    teleop = LaunchConfiguration("teleop")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model,
                description="G1 的 MuJoCo 场景文件路径",
            ),
            DeclareLaunchArgument(
                "weights",
                default_value=default_weights,
                description="TorchScript 策略权重路径",
            ),
            DeclareLaunchArgument(
                "teleop",
                default_value="true",
                description="是否启动键盘遥控节点（无终端时可设为 false）",
            ),
            Node(
                package="humanoid_ppo_locomotion",
                executable="sim_node",
                name="humanoid_sim_node",
                output="screen",
                parameters=[{"model_path": model_path}],
            ),
            Node(
                package="humanoid_ppo_locomotion",
                executable="policy_node",
                name="humanoid_policy_node",
                output="screen",
                parameters=[{"weights": weights}],
            ),
            Node(
                package="humanoid_ppo_locomotion",
                executable="keyboard_teleop",
                name="humanoid_keyboard_teleop",
                output="screen",
                prefix="xterm -e",
                condition=IfCondition(teleop),
            ),
        ]
    )
