"""Launch the VRU metrics node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ground_truth_csv", default_value=""),
        DeclareLaunchArgument("prediction_csv", default_value=""),
        DeclareLaunchArgument("max_distance", default_value="1.5"),
        Node(
            package="vru_twin_metrics",
            executable="vru_twin_metrics_node",
            name="vru_twin_metrics",
            output="screen",
            parameters=[{
                "ground_truth_csv": LaunchConfiguration("ground_truth_csv"),
                "prediction_csv": LaunchConfiguration("prediction_csv"),
                "max_distance": LaunchConfiguration("max_distance"),
            }],
        ),
    ])
