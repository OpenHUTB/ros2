"""启动 ROV 推力分配节点与期望力/力矩仿真。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("limit", default_value="10.0"),
        DeclareLaunchArgument("failed", default_value=""),
        DeclareLaunchArgument("csv_path", default_value="/tmp/rov_thruster_allocation_ls.csv"),
    ]
    sim = Node(package="rov_thruster_allocation_ls", executable="wrench_sim", name="wrench_sim",
               output="screen")
    ctrl = Node(package="rov_thruster_allocation_ls", executable="main", name="thruster_allocation",
                output="screen",
                parameters=[{"limit": LaunchConfiguration("limit"),
                             "failed": LaunchConfiguration("failed"),
                             "csv_path": LaunchConfiguration("csv_path")}])
    return LaunchDescription(args + [sim, ctrl])
