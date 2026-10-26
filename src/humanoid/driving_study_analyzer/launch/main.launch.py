"""Launch the offline driving-study analyzer node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("driving_csv", default_value=""),
        DeclareLaunchArgument("gaze_csv", default_value=""),
        DeclareLaunchArgument("output_json", default_value="driving_study_report.json"),
        Node(
            package="driving_study_analyzer",
            executable="driving_study_analyzer_node",
            name="driving_study_analyzer",
            output="screen",
            parameters=[{
                "driving_csv": LaunchConfiguration("driving_csv"),
                "gaze_csv": LaunchConfiguration("gaze_csv"),
                "output_json": LaunchConfiguration("output_json"),
            }],
        ),
    ])
