from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters = {
        "odom_topic": LaunchConfiguration("odom_topic"),
        "map_width": LaunchConfiguration("map_width"),
        "map_height": LaunchConfiguration("map_height"),
        "resolution": LaunchConfiguration("resolution"),
        "beam_count": LaunchConfiguration("beam_count"),
        "field_of_view": LaunchConfiguration("field_of_view"),
        "max_range": LaunchConfiguration("max_range"),
        "scan_rate": LaunchConfiguration("scan_rate"),
    }
    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("map_width", default_value="20.0"),
        DeclareLaunchArgument("map_height", default_value="20.0"),
        DeclareLaunchArgument("resolution", default_value="0.25"),
        DeclareLaunchArgument("beam_count", default_value="90"),
        DeclareLaunchArgument("field_of_view", default_value="6.283185307179586"),
        DeclareLaunchArgument("max_range", default_value="10.0"),
        DeclareLaunchArgument("scan_rate", default_value="5.0"),
        Node(
            package="underwater_sonar_mapping",
            executable="main",
            name="underwater_sonar_mapping_node",
            output="screen",
            parameters=[parameters],
        ),
    ])
