from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ego_csv", default_value=""),
        DeclareLaunchArgument("vru_csv", default_value=""),
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
