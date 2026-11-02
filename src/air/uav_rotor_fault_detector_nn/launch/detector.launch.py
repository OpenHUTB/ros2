"""``ros2 launch`` 入口：一条命令起仿真 + 检测 + 话题发布。

用法::

    ros2 launch uav_rotor_fault_detector_nn detector.launch.py
    ros2 launch uav_rotor_fault_detector_nn detector.launch.py fault:=degradation rotor:=2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "uav_rotor_fault_detector_nn"
EXECUTABLE = "detector_node"


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(share, "config", "detector.yaml")

    params_file = LaunchConfiguration("params_file")
    scene = LaunchConfiguration("scene")
    fault = LaunchConfiguration("fault")
    rotor = LaunchConfiguration("rotor")
    onset = LaunchConfiguration("onset")
    severity = LaunchConfiguration("severity")
    checkpoint = LaunchConfiguration("checkpoint")
    duration = LaunchConfiguration("duration")

    arguments = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="YAML 参数文件"),
        DeclareLaunchArgument("scene", default_value="open_ground",
                              description="仿真场景：open_ground/corridor/obstacle_field/urban_block"),
        DeclareLaunchArgument("fault", default_value="loss",
                              description="故障类型：healthy/loss/degradation/stuck/dual_loss/progressive"),
        DeclareLaunchArgument("rotor", default_value="0", description="故障桨序号 0-3"),
        DeclareLaunchArgument("onset", default_value="2.0", description="故障注入时刻（秒）"),
        DeclareLaunchArgument("severity", default_value="1.0", description="故障严重度"),
        DeclareLaunchArgument("checkpoint", default_value="results/detector.pt",
                              description="训练好的检查点路径"),
        DeclareLaunchArgument("duration", default_value="10.0", description="飞行时长（秒）"),
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable=EXECUTABLE,
        name=PACKAGE_NAME,
        output="screen",
        emulate_tty=True,
        parameters=[
            params_file,
            {
                "scene": ParameterValue(scene, value_type=str),
                "fault": ParameterValue(fault, value_type=str),
                "rotor": ParameterValue(rotor, value_type=int),
                "onset": ParameterValue(onset, value_type=float),
                "severity": ParameterValue(severity, value_type=float),
                "checkpoint": ParameterValue(checkpoint, value_type=str),
                "duration": ParameterValue(duration, value_type=float),
            },
        ],
    )

    return LaunchDescription(arguments + [node])
