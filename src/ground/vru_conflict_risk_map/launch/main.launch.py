import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("vru_conflict_risk_map")
    default_ego = os.path.join(package_share, "data", "ego.csv")
    default_vru = os.path.join(package_share, "data", "vru.csv")
    return LaunchDescription([
        DeclareLaunchArgument("ego_csv", default_value=default_ego),
        DeclareLaunchArgument("vru_csv", default_value=default_vru),
        Node(
            package="vru_conflict_risk_map",
            executable="vru_risk_map_node",
            name="vru_conflict_risk_map",
            output="screen",
            parameters=[{
                "ego_csv": LaunchConfiguration("ego_csv"),
                "vru_csv": LaunchConfiguration("vru_csv"),
            }],
        ),
    ])
