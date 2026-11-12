"""启动倒车纯追踪控制器与运动学仿真。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("speed", default_value="0.25"),
        DeclareLaunchArgument("lookahead", default_value="0.9"),
        DeclareLaunchArgument("max_steer_rate", default_value="1.2"),
        DeclareLaunchArgument("csv_path", default_value="/tmp/pure_pursuit_reverse.csv"),
    ]
    sim = Node(package="pure_pursuit_reverse", executable="path_sim", name="path_sim",
               output="screen", parameters=[{"csv_path": LaunchConfiguration("csv_path")}])
    ctrl = Node(package="pure_pursuit_reverse", executable="main", name="reverse_pursuit",
                output="screen",
                parameters=[{"speed": LaunchConfiguration("speed"),
                             "lookahead": LaunchConfiguration("lookahead"),
                             "max_steer_rate": LaunchConfiguration("max_steer_rate")}])
    return LaunchDescription(args + [sim, ctrl])
