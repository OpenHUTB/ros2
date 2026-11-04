"""Launch: sim_node + perception_node + avoidance_node (+ optional keyboard teleop).

Usage:
  ros2 launch humanoid_lidar_nn_perception main.launch.py
  ros2 launch humanoid_lidar_nn_perception main.launch.py teleop:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop = LaunchConfiguration("teleop")
    return LaunchDescription([
        DeclareLaunchArgument("teleop", default_value="true"),
        Node(
            package="humanoid_lidar_nn_perception",
            executable="sim_node",
            name="g1_sim_node",
            output="screen",
        ),
        Node(
            package="humanoid_lidar_nn_perception",
            executable="perception_node",
            name="perception_node",
            output="screen",
        ),
        Node(
            package="humanoid_lidar_nn_perception",
            executable="avoidance_node",
            name="avoidance_node",
            output="screen",
        ),
        Node(
            package="humanoid_lidar_nn_perception",
            executable="keyboard_teleop",
            name="keyboard_teleop",
            output="screen",
            prefix="xterm -e",
            condition=IfCondition(teleop),
        ),
    ])
