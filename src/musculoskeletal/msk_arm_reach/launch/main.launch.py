"""ros2 launch mujoco_musculoskeletal_arm_reach main.launch.py"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    x = LaunchConfiguration("target_x")
    z = LaunchConfiguration("target_z")
    seconds = LaunchConfiguration("seconds")

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_x", default_value="0.38"),
            DeclareLaunchArgument("target_z", default_value="-0.17"),
            DeclareLaunchArgument("seconds", default_value="0.0",
                                  description=">0 时运行该秒数后自动退出"),
            Node(
                package="mujoco_musculoskeletal_arm_reach",
                executable="arm_sim_node",
                name="mujoco_musculoskeletal_arm_reach",
                output="screen",
                parameters=[
                    {
                        "target_x": ParameterValue(x, value_type=float),
                        "target_z": ParameterValue(z, value_type=float),
                        "seconds": ParameterValue(seconds, value_type=float),
                    }
                ],
            ),
        ]
    )
