"""通过 ROS 2 launch 启动 MuJoCo 单臂抓取演示。

用法：
  ros2 launch pi0_mujoco_grasp main.launch.py                       # 仅仿真
  ros2 launch pi0_mujoco_grasp main.launch.py record:=/tmp/pi0_frames fps:=30
  ros2 launch pi0_mujoco_grasp main.launch.py policy:=pi0            # 需要 GPU
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record", default_value="",
                description="渲染帧序列的输出目录；为空则只仿真不渲染"),
            DeclareLaunchArgument(
                "fps", default_value="30",
                description="渲染帧率（默认 30）"),
            DeclareLaunchArgument(
                "policy", default_value="stub",
                description="stub = CPU 脚本策略替身；pi0 = 真实 pi0 推理（需 GPU）"),
            Node(
                package="pi0_mujoco_grasp",
                executable="pi0_mujoco_grasp",
                name="pi0_mujoco_grasp",
                output="screen",
                arguments=[
                    "--record", LaunchConfiguration("record"),
                    "--fps", LaunchConfiguration("fps"),
                    "--policy", LaunchConfiguration("policy"),
                ],
            ),
        ]
    )
