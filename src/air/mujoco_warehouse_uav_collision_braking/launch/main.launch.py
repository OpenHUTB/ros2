"""启动完整有限时长仿真。 / Launch the complete finite-duration simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="static"),
        DeclareLaunchArgument("duration", default_value="8.0"),
        DeclareLaunchArgument("output", default_value="/tmp/warehouse_braking"),
        Node(package="mujoco_warehouse_uav_collision_braking", executable="sensor_node", output="screen",
             parameters=[{"scenario": LaunchConfiguration("scenario"),
                          "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
                          "output": LaunchConfiguration("output")}])])
