from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_rate = ParameterValue(LaunchConfiguration("sim_rate"), value_type=float)
    max_steps = ParameterValue(LaunchConfiguration("max_steps"), value_type=int)

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_rate", default_value="60.0",
            description="仿真推进频率（Hz），同时决定位姿发布频率"),
        DeclareLaunchArgument(
            "max_steps", default_value="3000",
            description="最大仿真步数，到达上限后节点停止推进"),
        Node(
            package="potential_field_navigation",
            executable="potential_field_node",
            name="potential_field_navigation",
            output="screen",
            parameters=[{"sim_rate": sim_rate, "max_steps": max_steps}],
        ),
    ])
