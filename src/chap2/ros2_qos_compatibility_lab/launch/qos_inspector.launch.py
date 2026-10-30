"""Start the QoS inspector without the demo endpoints."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the standalone inspector launch description."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "topic_filter",
                default_value="",
                description="Only inspect topic names containing this text",
            ),
            DeclareLaunchArgument(
                "scan_period_sec",
                default_value="2.0",
            ),
            Node(
                package="ros2_qos_compatibility_lab",
                executable="qos_inspector",
                name="qos_compatibility_inspector",
                output="screen",
                parameters=[
                    {
                        "topic_filter": ParameterValue(
                            LaunchConfiguration("topic_filter"),
                            value_type=str,
                        ),
                        "scan_period_sec": ParameterValue(
                            LaunchConfiguration("scan_period_sec"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
