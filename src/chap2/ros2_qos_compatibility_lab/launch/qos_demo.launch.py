"""Start an intentionally incompatible QoS demo and the inspector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the QoS demo launch description."""

    topic = LaunchConfiguration("topic")
    publisher_reliability = LaunchConfiguration("publisher_reliability")
    subscriber_reliability = LaunchConfiguration("subscriber_reliability")
    publisher_durability = LaunchConfiguration("publisher_durability")
    subscriber_durability = LaunchConfiguration("subscriber_durability")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "topic",
                default_value="/qos_lab/chatter",
            ),
            DeclareLaunchArgument(
                "publisher_reliability",
                default_value="best_effort",
            ),
            DeclareLaunchArgument(
                "subscriber_reliability",
                default_value="reliable",
            ),
            DeclareLaunchArgument(
                "publisher_durability",
                default_value="volatile",
            ),
            DeclareLaunchArgument(
                "subscriber_durability",
                default_value="volatile",
            ),
            Node(
                package="ros2_qos_compatibility_lab",
                executable="demo_publisher",
                name="qos_demo_publisher",
                output="screen",
                parameters=[
                    {
                        "topic": ParameterValue(topic, value_type=str),
                        "reliability": ParameterValue(
                            publisher_reliability,
                            value_type=str,
                        ),
                        "durability": ParameterValue(
                            publisher_durability,
                            value_type=str,
                        ),
                    }
                ],
            ),
            Node(
                package="ros2_qos_compatibility_lab",
                executable="demo_subscriber",
                name="qos_demo_subscriber",
                output="screen",
                parameters=[
                    {
                        "topic": ParameterValue(topic, value_type=str),
                        "reliability": ParameterValue(
                            subscriber_reliability,
                            value_type=str,
                        ),
                        "durability": ParameterValue(
                            subscriber_durability,
                            value_type=str,
                        ),
                    }
                ],
            ),
            Node(
                package="ros2_qos_compatibility_lab",
                executable="qos_inspector",
                name="qos_compatibility_inspector",
                output="screen",
                parameters=[
                    {
                        "scan_period_sec": 1.0,
                        "topic_filter": ParameterValue(topic, value_type=str),
                    }
                ],
            ),
        ]
    )
