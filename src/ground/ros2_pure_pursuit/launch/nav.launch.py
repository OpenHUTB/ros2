"""ROS2 启动文件：启动纯追踪导航节点。

    ros2 launch ros2_pure_pursuit nav.launch.py
    ros2 launch ros2_pure_pursuit nav.launch.py target_speed:=1.0 enable_avoidance:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("ros2_pure_pursuit")

    default_path = PathJoinSubstitution([pkg_share, "paths", "circle.csv"])

    args = [
        DeclareLaunchArgument(
            "target_speed",
            default_value="0.8",
            description="期望线速度 (m/s)",
        ),
        DeclareLaunchArgument(
            "path_file",
            default_value=default_path,
            description="参考路径 csv 文件路径",
        ),
        DeclareLaunchArgument(
            "enable_avoidance",
            default_value="true",
            description="是否启用激光避障层",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="节点命名空间",
        ),
    ]

    node = Node(
        package="ros2_pure_pursuit",
        executable="pure_pursuit_node",
        name="pure_pursuit_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "target_speed": LaunchConfiguration("target_speed"),
                "path_file": LaunchConfiguration("path_file"),
                "enable_avoidance": LaunchConfiguration("enable_avoidance"),
            }
        ],
    )

    return LaunchDescription(args + [node])
