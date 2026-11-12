"""启动端到端推力策略节点。

参数分两层：``config/policy.yaml`` 是默认值真源，这里的 launch 参数只覆盖常用的几个；
两处的默认值（以及 ``ros_node.DEFAULT_PARAMETERS``）由 test/test_entry.py 比对，
避免出现"改了 yaml 但 launch 又盖回去"的假象。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "uav_fault_e2e_thrust_policy"
NODE_NAME = "uav_fault_e2e_thrust_policy"


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare(PACKAGE_NAME), "config", "policy.yaml"])

    args = [
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="参数文件路径（默认取包内 config/policy.yaml）"),
        DeclareLaunchArgument("fault", default_value="degradation",
                              description="注入的桨故障类型（看基线用 healthy）"),
        DeclareLaunchArgument("rotor", default_value="2",
                              description="故障桨序号 0~3"),
        DeclareLaunchArgument("onset", default_value="2.5",
                              description="故障注入时刻 s"),
        DeclareLaunchArgument("severity", default_value="0.5",
                              description="故障严重程度（衰减/渐进用）"),
        DeclareLaunchArgument("duration", default_value="30.0",
                              description="仿真时长 s"),
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="policy_node",
        name=NODE_NAME,
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "fault": LaunchConfiguration("fault"),
                "rotor": ParameterValue(LaunchConfiguration("rotor"), value_type=int),
                "onset": ParameterValue(LaunchConfiguration("onset"), value_type=float),
                "severity": ParameterValue(LaunchConfiguration("severity"), value_type=float),
                "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
            },
        ],
    )

    return LaunchDescription(args + [node])
