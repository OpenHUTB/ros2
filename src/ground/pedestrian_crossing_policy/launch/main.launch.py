"""Launch the pedestrian crossing policy node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("min_time_to_collision", default_value="3.0"),
        DeclareLaunchArgument("max_vehicle_distance", default_value="12.0"),
        DeclareLaunchArgument("min_vehicle_distance", default_value="1.5"),
        Node(
            package="pedestrian_crossing_policy",
            executable="crossing_policy_node",
            name="pedestrian_crossing_policy",
            output="screen",
            parameters=[{
                "min_time_to_collision": LaunchConfiguration("min_time_to_collision"),
                "max_vehicle_distance": LaunchConfiguration("max_vehicle_distance"),
                "min_vehicle_distance": LaunchConfiguration("min_vehicle_distance"),
            }],
        ),
    ])
