"""启动 A* 全局规划节点。

    ros2 launch ros2_astar_planner plan.launch.py
    ros2 launch ros2_astar_planner plan.launch.py robot_radius:=0.25 use_smoothing:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("robot_radius", default_value="0.20"),
        DeclareLaunchArgument("inflation_extra", default_value="0.0"),
        DeclareLaunchArgument("allow_diagonal", default_value="true"),
        DeclareLaunchArgument("heuristic", default_value="octile"),
        DeclareLaunchArgument("weight", default_value="1.0"),
        DeclareLaunchArgument("v_max", default_value="0.6"),
        DeclareLaunchArgument("a_max", default_value="0.5"),
        DeclareLaunchArgument("plan_timeout", default_value="2.0"),
        DeclareLaunchArgument("map_timeout", default_value="5.0"),
        DeclareLaunchArgument("use_smoothing", default_value="true"),
        DeclareLaunchArgument("node_name", default_value="astar_planner"),
    ]

    planner = Node(
        package="ros2_astar_planner",
        executable="astar_planner_node",
        name=LaunchConfiguration("node_name"),
        output="screen",
        parameters=[
            {
                "robot_radius": LaunchConfiguration("robot_radius"),
                "inflation_extra": LaunchConfiguration("inflation_extra"),
                "allow_diagonal": LaunchConfiguration("allow_diagonal"),
                "heuristic": LaunchConfiguration("heuristic"),
                "weight": LaunchConfiguration("weight"),
                "v_max": LaunchConfiguration("v_max"),
                "a_max": LaunchConfiguration("a_max"),
                "plan_timeout": LaunchConfiguration("plan_timeout"),
                "map_timeout": LaunchConfiguration("map_timeout"),
                "use_smoothing": LaunchConfiguration("use_smoothing"),
            }
        ],
    )

    return LaunchDescription(args + [planner])
