"""``ros2 launch`` 入口：一条命令起仿真 + 检测 + 话题发布。

用法::

    ros2 launch uav_rotor_fault_detector_nn detector.launch.py
    ros2 launch uav_rotor_fault_detector_nn detector.launch.py fault:=degradation rotor:=2

参数分两层，避免"两个地方都能改同一项"的歧义：

* ``config/detector.yaml`` —— 默认值真源，13 个参数全在里面；
* 本文件只把演示时最常改的 6 个参数做成 launch argument，传递时覆盖 yaml 的同名项。
  ``seed`` / ``dt`` / ``substeps`` / ``imu_noise`` / ``checkpoint`` / ``window`` /
  ``publish_rate`` 不在这里暴露，要改就去改 yaml 或用 ``--ros-args -p``。
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
NODE_NAME = "uav_rotor_fault_detector_nn"


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE_NAME)
    default_params = os.path.join(share, "config", "detector.yaml")

    params_file = LaunchConfiguration("params_file")
    scene = LaunchConfiguration("scene")
    fault = LaunchConfiguration("fault")
    rotor = LaunchConfiguration("rotor")
    onset = LaunchConfiguration("onset")
    severity = LaunchConfiguration("severity")
    duration = LaunchConfiguration("duration")

    arguments = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="YAML 参数文件（默认值真源）"),
        DeclareLaunchArgument("scene", default_value="open_ground",
                              description="仿真场景：open_ground/corridor/obstacle_field/urban_block"),
        DeclareLaunchArgument("fault", default_value="loss",
                              description="故障类型：healthy/loss/degradation/stuck/dual_loss/progressive"),
        DeclareLaunchArgument("rotor", default_value="0", description="故障桨序号 0-3"),
        DeclareLaunchArgument("onset", default_value="2.0", description="故障注入时刻（秒）"),
        DeclareLaunchArgument("severity", default_value="1.0", description="故障严重度"),
        DeclareLaunchArgument("duration", default_value="10.0", description="飞行时长（秒）"),
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable=EXECUTABLE,
        name=NODE_NAME,
        output="screen",
        emulate_tty=True,
        # 后面的字典只列上面暴露的 6 项，其余参数由 params_file 提供
        parameters=[
            params_file,
            {
                "scene": ParameterValue(scene, value_type=str),
                "fault": ParameterValue(fault, value_type=str),
                "rotor": ParameterValue(rotor, value_type=int),
                "onset": ParameterValue(onset, value_type=float),
                "severity": ParameterValue(severity, value_type=float),
                "duration": ParameterValue(duration, value_type=float),
            },
        ],
    )

    return LaunchDescription(arguments + [node])
