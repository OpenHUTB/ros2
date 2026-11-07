"""humanoid_h1_walk 一键启动。

    ros2 launch humanoid_h1_walk main.launch.py
    ros2 launch humanoid_h1_walk main.launch.py robot:=h1_2
    ros2 launch humanoid_h1_walk main.launch.py routine:=true          # 固定动作序列
    ros2 launch humanoid_h1_walk main.launch.py teleop:=false          # 不启键盘节点
    ros2 launch humanoid_h1_walk main.launch.py policy_in_sim:=true    # 单进程推理

启动内容：
    1. sim_node         MuJoCo 物理 + 观测，发布 /joint_states、/odom、/humanoid/obs
    2. policy_node      TorchScript 策略推理，/humanoid/obs → /humanoid/joint_targets
    3. keyboard_teleop  方向键 → /cmd_vel（默认开启，需要可交互终端）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot = LaunchConfiguration("robot")
    config = LaunchConfiguration("config")
    routine = LaunchConfiguration("routine")
    teleop = LaunchConfiguration("teleop")
    policy_in_sim = LaunchConfiguration("policy_in_sim")
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="h1", description="机型：h1 或 h1_2"),
            DeclareLaunchArgument("config", default_value="", description="自定义 robots.yaml 路径"),
            DeclareLaunchArgument("routine", default_value="false", description="播放固定动作序列"),
            DeclareLaunchArgument("teleop", default_value="true", description="是否启动键盘遥控节点"),
            DeclareLaunchArgument(
                "policy_in_sim",
                default_value="false",
                description="在仿真节点内直接推理（单进程，等价上游 deploy_mujoco.py）",
            ),
            DeclareLaunchArgument(
                "publish_tf", default_value="true", description="是否发布 odom→base_link"
            ),
            Node(
                package="humanoid_h1_walk",
                executable="sim_node",
                name="humanoid_h1_walk_sim",
                output="screen",
                parameters=[
                    {
                        "robot": robot,
                        "config": config,
                        "routine": routine,
                        "policy_in_sim": policy_in_sim,
                        "publish_tf": publish_tf,
                    }
                ],
            ),
            Node(
                package="humanoid_h1_walk",
                executable="policy_node",
                name="humanoid_h1_walk_policy",
                output="screen",
                parameters=[{"robot": robot, "config": config}],
                condition=UnlessCondition(policy_in_sim),
            ),
            Node(
                package="humanoid_h1_walk",
                executable="keyboard_teleop",
                name="humanoid_h1_walk_teleop",
                output="screen",
                condition=IfCondition(teleop),
            ),
        ]
    )
