import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("vru_conflict_risk_map")
    default_ego = os.path.join(package_share, "data", "ego.csv")
    default_vru = os.path.join(package_share, "data", "vru.csv")
    return LaunchDescription([
        DeclareLaunchArgument("ego_csv", default_value=default_ego),
        DeclareLaunchArgument("vru_csv", default_value=default_vru),
        DeclareLaunchArgument("analysis_mode", default_value="point"),
        DeclareLaunchArgument("ego_radius", default_value="1.0"),
        DeclareLaunchArgument("caution_clearance", default_value="1.0"),
        DeclareLaunchArgument("horizon", default_value="5.0"),
        Node(
            package="vru_conflict_risk_map",
            executable="vru_risk_map_node",
            name="vru_conflict_risk_map",
            output="screen",
            parameters=[{
                "ego_csv": LaunchConfiguration("ego_csv"),
                "vru_csv": LaunchConfiguration("vru_csv"),
                "analysis_mode": LaunchConfiguration("analysis_mode"),
                "ego_radius": ParameterValue(
                    LaunchConfiguration("ego_radius"), value_type=float),
                "caution_clearance": ParameterValue(
                    LaunchConfiguration("caution_clearance"), value_type=float),
                "horizon": ParameterValue(
                    LaunchConfiguration("horizon"), value_type=float),
            }],
        ),
    ])
