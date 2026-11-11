import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    tb3_models = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "models")
    world = os.path.join(get_package_share_directory("ros2_dwa_gazebo"),
                         "worlds", "dwa_corridor.world")

    gzserver = ExecuteProcess(
        cmd=["gzserver", "--verbose", world,
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    max_v = LaunchConfiguration("max_v")

    dwa = Node(
        package="ros2_dwa_avoidance",
        executable="main_dwa",
        name="dwa_avoidance",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "max_v": max_v,
            "max_w": 1.0,
            "max_acc_v": 0.8,
            "max_acc_w": 2.0,
            "predict_time": 1.5,
            "robot_radius": 0.12,
            "safety_margin": 0.04,
            "goal_tolerance": 0.20,
            "map_half_size": 6.0,
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            tb3_models + ":/usr/share/gazebo-11/models:" +
            os.environ.get("GAZEBO_MODEL_PATH", "")),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        DeclareLaunchArgument("goal_x", default_value="2.5"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("max_v", default_value="0.22"),
        gzserver,
        dwa,
    ])
