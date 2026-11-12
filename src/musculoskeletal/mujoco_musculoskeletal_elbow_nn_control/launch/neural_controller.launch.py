"""Launch the elbow neural controller."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([Node(
        package="mujoco_musculoskeletal_elbow_nn_control",
        executable="neural_elbow_controller", name="mujoco_elbow_neural_controller",
        output="screen", parameters=[PathJoinSubstitution([
            FindPackageShare("mujoco_musculoskeletal_elbow_nn_control"),
            "config", "neural_controller.yaml",
        ])],
    )])
