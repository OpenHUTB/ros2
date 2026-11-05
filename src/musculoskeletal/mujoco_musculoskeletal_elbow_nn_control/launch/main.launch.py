from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "mujoco_musculoskeletal_elbow_nn_control"
    share = Path(get_package_share_directory(package_name))
    return LaunchDescription(
        [
            Node(
                package=package_name,
                executable="elbow_sim_node",
                name="musculoskeletal_elbow_simulation",
                output="screen",
                parameters=[str(share / "config" / "simulation.yaml")],
            )
        ]
    )
