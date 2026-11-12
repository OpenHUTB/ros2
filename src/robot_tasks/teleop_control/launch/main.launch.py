from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("linear_speed", default_value="0.5"),
            DeclareLaunchArgument("angular_speed", default_value="0.5"),
            DeclareLaunchArgument("command_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("poll_period_sec", default_value="0.05"),
            Node(
                package="teleop_control",
                executable="main_teleop",
                name="teleop_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "linear_speed": ParameterValue(
                            LaunchConfiguration("linear_speed"), value_type=float
                        ),
                        "angular_speed": ParameterValue(
                            LaunchConfiguration("angular_speed"), value_type=float
                        ),
                        "command_timeout_sec": ParameterValue(
                            LaunchConfiguration("command_timeout_sec"), value_type=float
                        ),
                        "poll_period_sec": ParameterValue(
                            LaunchConfiguration("poll_period_sec"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
