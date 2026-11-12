"""启动 LOS 制导节点与无人机质点仿真。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("uav_los_waypoint_guidance")
    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([pkg_share, "config", "params.yaml"]),
            description="ROS 2 参数文件（节点名下的 ros__parameters）"),
        DeclareLaunchArgument("csv_path", default_value="/tmp/uav_los_waypoint_guidance.csv"),
    ]
    sim = Node(package="uav_los_waypoint_guidance", executable="uav_sim", name="uav_sim",
               output="screen", parameters=[{"csv_path": LaunchConfiguration("csv_path")}])
    # 直接把 yaml 交给 ROS 2 参数系统加载，命令行 -p 覆盖也会生效
    ctrl = Node(package="uav_los_waypoint_guidance", executable="main",
                name="uav_los_waypoint_guidance", output="screen",
                parameters=[LaunchConfiguration("params_file")])
    return LaunchDescription(args + [sim, ctrl])
