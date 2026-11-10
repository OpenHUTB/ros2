import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("uav_esc_telemetry_watch"), "config", "demo.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("duration", default_value="0.0"),
        Node(package="uav_esc_telemetry_watch", executable="esc_sim", name="uav_esc_sim",
             output="screen", parameters=[cfg, {"duration_s": ParameterValue(LaunchConfiguration("duration"), value_type=float)}]),
        Node(package="uav_esc_telemetry_watch", executable="esc_watch", name="uav_esc_watch",
             output="screen", parameters=[cfg]),
    ])
