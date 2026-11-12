from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scene = LaunchConfiguration("scene")
    module = LaunchConfiguration("module")
    rate = LaunchConfiguration("rate")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")

    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="sparse",
                              description="sparse | dense | dynamic | corridor"),
        DeclareLaunchArgument("module", default_value="full",
                              description="perception | planning | control | e2e | full"),
        DeclareLaunchArgument("rate", default_value="10.0"),
        DeclareLaunchArgument("goal_x", default_value="49.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        Node(package="neural_ca_drone_ros2", executable="sim_bridge.py",
             name="ca_sim_bridge", output="screen",
             parameters=[{"scene": scene, "rate": rate}]),
        Node(package="neural_ca_drone_ros2", executable="main.py",
             name="neural_ca_drone", output="screen",
             parameters=[{"module": module, "rate": rate, "goal_x": goal_x,
                          "goal_y": goal_y, "replan_period": 0.5}]),
    ])
