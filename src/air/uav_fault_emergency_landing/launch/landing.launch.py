"""启动故障迫降节点。

参数默认值来自 config/landing.yaml；下面这几个是调参时最常改的，单独暴露成 launch 参数，
优先级高于 yaml。用法：

    ros2 launch uav_fault_emergency_landing landing.launch.py mode:=neural rotor:=1
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "uav_fault_emergency_landing"
NODE_NAME = "uav_fault_emergency_landing"


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [get_package_share_directory(PACKAGE_NAME), "config", "landing.yaml"])

    args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="参数文件路径"),
        DeclareLaunchArgument("scene", default_value="urban_block",
                              description="仿真场景名"),
        DeclareLaunchArgument("fault", default_value="degradation",
                              description="注入的故障类型"),
        DeclareLaunchArgument("rotor", default_value="2", description="故障桨序号 0-3"),
        DeclareLaunchArgument("severity", default_value="0.5",
                              description="退化后的推力保留比例"),
        DeclareLaunchArgument("duration", default_value="40.0",
                              description="最长仿真时长（秒）"),
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="landing_node",
        name=NODE_NAME,
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "scene": LaunchConfiguration("scene"),
                "fault": LaunchConfiguration("fault"),
                "rotor": ParameterValue(LaunchConfiguration("rotor"), value_type=int),
                "severity": ParameterValue(LaunchConfiguration("severity"),
                                           value_type=float),
                "duration": ParameterValue(LaunchConfiguration("duration"),
                                           value_type=float),
            },
        ],
    )
    return LaunchDescription(args + [node])
