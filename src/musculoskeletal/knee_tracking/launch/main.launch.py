"""ros2 launch mujoco_musculoskeletal_knee_tracking main.launch.py"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    duration = LaunchConfiguration("duration")
    traj = LaunchConfiguration("trajectory_type")
    bias = LaunchConfiguration("bias_deg")
    exit_after = LaunchConfiguration("exit_after_duration")

    return LaunchDescription(
        [
            DeclareLaunchArgument("duration", default_value="30.0",
                                  description="内置轨迹时长 [s]"),
            DeclareLaunchArgument("trajectory_type", default_value="sine",
                                  choices=["sine", "step"]),
            DeclareLaunchArgument("bias_deg", default_value="30.0"),
            DeclareLaunchArgument("exit_after_duration", default_value="false",
                                  description="内置轨迹跑完后节点自动退出"),
            Node(
                package="mujoco_musculoskeletal_knee_tracking",
                executable="mujoco_musculoskeletal_knee_tracking_node",
                name="mujoco_musculoskeletal_knee_tracking",
                output="screen",
                parameters=[
                    {
                        "duration": ParameterValue(duration, value_type=float),
                        "trajectory_type": ParameterValue(traj, value_type=str),
                        "bias_deg": ParameterValue(bias, value_type=float),
                        "exit_after_duration": ParameterValue(
                            exit_after, value_type=bool),
                    }
                ],
            ),
        ]
    )
