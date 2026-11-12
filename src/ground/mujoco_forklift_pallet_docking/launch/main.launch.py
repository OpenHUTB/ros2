from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackagePrefix
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    executable = PathJoinSubstitution([
        FindPackagePrefix("mujoco_forklift_pallet_docking"),
        "lib", "mujoco_forklift_pallet_docking", "forklift_pallet_docking",
    ])
    return LaunchDescription([
        ExecuteProcess(cmd=[executable, "--mode", "evaluate"], output="screen"),
    ])
