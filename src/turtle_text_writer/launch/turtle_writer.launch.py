import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    main_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "main.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument("text", default_value="HUTB", description="要书写的文字"),
        ExecuteProcess(
            cmd=["ros2", "run", "turtlesim", "turtlesim_node"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", main_path, "--text", LaunchConfiguration("text")],
            output="screen",
        ),
    ])
