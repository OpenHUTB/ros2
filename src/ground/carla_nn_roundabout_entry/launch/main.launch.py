from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "metrics_path",
            default_value="/tmp/roundabout_navigation_metrics.json",
            description="File path for final navigation metrics",
        ),
        Node(
            package="carla_nn_roundabout_entry",
            executable="roundabout_simulator",
            name="roundabout_simulator",
            output="screen",
        ),
        Node(
            package="carla_nn_roundabout_entry",
            executable="roundabout_navigation",
            name="roundabout_navigation",
            output="screen",
            parameters=[{"metrics_path": LaunchConfiguration("metrics_path")}],
        ),
    ])
