"""容错控制节点的启动文件。

两层的分工（PR #604 的教训：同一个参数只能有一个真源）：

* ``config/control.yaml`` 是默认值真源，包含全部节点参数；
* 这里只把常改的几项做成 launch 参数，覆盖 yaml。

用法::

    ros2 launch uav_neural_fault_tolerant_control control.launch.py
    ros2 launch uav_neural_fault_tolerant_control control.launch.py \\
        fault:=loss rotor:=1 onset:=3.0 duration:=15.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "uav_neural_fault_tolerant_control"
NODE_NAME = "uav_neural_fault_tolerant_control"


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare(PACKAGE_NAME), "config", "control.yaml"]),
            description="节点参数文件（默认值真源）"),
        DeclareLaunchArgument("scene", default_value="open_ground",
                              description="仿真场景"),
        DeclareLaunchArgument("fault", default_value="degradation",
                              description="故障类型"),
        DeclareLaunchArgument("rotor", default_value="1",
                              description="故障桨序号 0~3"),
        DeclareLaunchArgument("onset", default_value="2.0",
                              description="故障注入时刻 s"),
        DeclareLaunchArgument("severity", default_value="0.5",
                              description="故障严重程度"),
        DeclareLaunchArgument("duration", default_value="12.0",
                              description="运行时长 s"),
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="control_node",
        name=NODE_NAME,
        output="screen",
        emulate_tty=True,
        parameters=[
            params_file,
            {
                "scene": ParameterValue(LaunchConfiguration("scene"), value_type=str),
                "fault": ParameterValue(LaunchConfiguration("fault"), value_type=str),
                "rotor": ParameterValue(LaunchConfiguration("rotor"), value_type=int),
                "onset": ParameterValue(LaunchConfiguration("onset"), value_type=float),
                "severity": ParameterValue(LaunchConfiguration("severity"),
                                           value_type=float),
                "duration": ParameterValue(LaunchConfiguration("duration"),
                                           value_type=float),
            },
        ],
    )

    return LaunchDescription(args + [node])
