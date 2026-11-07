import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rov_wreck_inspection")
    world_file = os.path.join(pkg_share, "world", "wreck_inspection.world")
    urdf_file = os.path.join(pkg_share, "urdf", "rov.urdf")
    rviz_file = os.path.join(pkg_share, "rviz", "wreck_inspection.rviz")
    cfg_file = os.path.join(pkg_share, "config", "inspection.yaml")

    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("rviz")

    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_desc = f.read()

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

    # 出生点与巡检椭圆参数角 t=0 处一致，避免巡检节点接管时模型跳变
    spawn_rov = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "rov",
            "-x", "6.6", "-y", "0.0", "-z", "-3.6",
            "-R", "0.0", "-P", "0.0", "-Y", "3.14159",
            "-timeout", "60",
        ],
        output="screen",
    )

    detector = Node(
        package="rov_wreck_inspection",
        executable="hull_detector.py",
        name="hull_detector",
        output="screen",
        parameters=[{"config_file": cfg_file, "use_sim_time": True}],
    )

    controller = Node(
        package="rov_wreck_inspection",
        executable="inspection_controller.py",
        name="inspection_controller",
        output="screen",
        parameters=[{"config_file": cfg_file, "use_sim_time": True}],
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
        DeclareLaunchArgument("gui", default_value="true",
                              description="启动 Gazebo 图形界面"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="启动 RViz2"),
        gazebo,
        robot_state_publisher,
        TimerAction(period=10.0, actions=[spawn_rov]),
        TimerAction(period=14.0, actions=[detector, controller]),
        TimerAction(period=16.0, actions=[rviz]),
    ])
