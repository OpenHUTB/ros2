"""启动禁飞区绕行节点与无人机仿真。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("uav_geofence_bypass")
    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([pkg_share, "config", "params.yaml"]),
            description="禁飞区与规划参数文件"),
        DeclareLaunchArgument("csv_path", default_value="/tmp/uav_geofence_bypass.csv"),
    ]
    sim = Node(package="uav_geofence_bypass", executable="uav_sim", name="uav_sim",
               output="screen", parameters=[{"csv_path": LaunchConfiguration("csv_path")}])
    ctrl = Node(package="uav_geofence_bypass", executable="main",
                name="uav_geofence_bypass", output="screen",
                parameters=[{"params_file": LaunchConfiguration("params_file")}])
    return LaunchDescription(args + [sim, ctrl])
