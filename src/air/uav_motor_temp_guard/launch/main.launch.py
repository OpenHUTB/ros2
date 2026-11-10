import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("uav_motor_temp_guard"), "config", "demo.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("duration", default_value="0.0"),
        Node(package="uav_motor_temp_guard", executable="motor_sim", name="uav_motor_sim",
             output="screen", parameters=[cfg, {"duration_s": ParameterValue(LaunchConfiguration("duration"), value_type=float)}]),
        Node(package="uav_motor_temp_guard", executable="motor_temp_guard", name="uav_motor_temp_guard",
             output="screen", parameters=[cfg]),
    ])
