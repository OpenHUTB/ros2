import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rov_apriltag_cruise")
    world_file = os.path.join(pkg_share, "world", "underwater_apriltag.world")
    urdf_file = os.path.join(pkg_share, "urdf", "rov.urdf")
    rviz_file = os.path.join(pkg_share, "rviz", "rov_apriltag_cruise.rviz")
    models_dir = os.path.join(pkg_share, "models")

    gui = LaunchConfiguration("gui")
    show_camera_gui = LaunchConfiguration("show_camera_gui")
    use_rviz = LaunchConfiguration("rviz")

    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_desc = f.read()

    set_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        models_dir + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"),
                         "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": world_file,
            "gui": gui,
            "verbose": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    spawn_rov = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "rov",
            "-x", "0.0", "-y", "0.0", "-z", "-2.2",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0",
        ],
        output="screen",
    )

    detector = Node(
        package="rov_apriltag_cruise",
        executable="apriltag_detector_node",
        name="apriltag_detector",
        output="screen",
        parameters=[{
            "marker_size": 0.6,
            "show_gui": show_camera_gui,
            "use_sim_time": True,
        }],
    )

    cruise = Node(
        package="rov_apriltag_cruise",
        executable="cruise_controller.py",
        name="cruise_controller",
        output="screen",
        parameters=[{
            "config_file": os.path.join(pkg_share, "config", "cruise.yaml"),
            "use_sim_time": True,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="启动 Gazebo 图形界面"),
        DeclareLaunchArgument("show_camera_gui", default_value="true",
                              description="弹出 ROV 相机识别窗口"),
        DeclareLaunchArgument("rviz", default_value="true", description="启动 RViz"),
        set_model_path,
        gazebo,
        robot_state_publisher,
        TimerAction(period=4.0, actions=[spawn_rov]),
        TimerAction(period=8.0, actions=[detector, cruise]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
