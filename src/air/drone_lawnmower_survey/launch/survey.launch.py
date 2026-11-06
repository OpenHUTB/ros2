"""Launch the survey CLI through ROS 2, previewing by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import FindExecutable, LaunchConfiguration


ARGUMENTS = (
    ("length", "60"),
    ("width", "40"),
    ("lane_spacing", "8"),
    ("altitude", "20"),
    ("speed", "5"),
    ("ip", "127.0.0.1"),
    ("port", "41451"),
    ("vehicle", ""),
    ("execute", "false"),
)


def _command():
    command = [FindExecutable(name="survey")]
    for name in ARGUMENTS[:-1]:
        command.extend((f"--{name[0].replace('_', '-')}", LaunchConfiguration(name[0])))
    return command


def generate_launch_description():
    """Return a reproducible preview/execute launch description."""

    declarations = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in ARGUMENTS
    ]
    command = _command()
    return LaunchDescription(
        declarations
        + [
            ExecuteProcess(
                cmd=command,
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("execute")),
            ),
            ExecuteProcess(
                cmd=command + ["--execute"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("execute")),
            ),
        ]
    )
