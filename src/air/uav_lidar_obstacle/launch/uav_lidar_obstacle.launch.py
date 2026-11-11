# -*- coding: utf-8 -*-
"""启动无人机激光雷达避障节点。

用法::

    ros2 launch uav_lidar_obstacle uav_lidar_obstacle.launch.py

前提：点云话题已有发布者，例如先启动 carla_ros_bridge 并生成带 lidar 的无人机
（本包的纯 CARLA 脚本 ``drone_control.py`` 是另一条不依赖 ROS 的路径）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lidar_topic = LaunchConfiguration("lidar_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    stop_distance = LaunchConfiguration("stop_distance")
    forward_speed = LaunchConfiguration("forward_speed")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    data_timeout_s = LaunchConfiguration("data_timeout_s")

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_topic", default_value="/carla/lidar/point_cloud",
            description="机载激光雷达点云话题（sensor_msgs/PointCloud2）"),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="/uav/cmd_vel",
            description="速度指令话题（geometry_msgs/Twist）"),
        DeclareLaunchArgument(
            "stop_distance", default_value="4.0",
            description="停车阈值（米）"),
        DeclareLaunchArgument(
            "forward_speed", default_value="2.0",
            description="无障碍物时的前飞速度（米/秒）"),
        DeclareLaunchArgument(
            "control_rate_hz", default_value="10.0",
            description="控制频率（赫兹）"),
        DeclareLaunchArgument(
            "data_timeout_s", default_value="1.0",
            description="点云超时（秒），超时按故障安全停车"),

        Node(
            package="uav_lidar_obstacle",
            executable="uav_lidar_obstacle_node",
            name="uav_lidar_obstacle",
            output="screen",
            parameters=[{
                "lidar_topic": lidar_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "stop_distance": ParameterValue(stop_distance,
                                                value_type=float),
                "forward_speed": ParameterValue(forward_speed,
                                                value_type=float),
                "control_rate_hz": ParameterValue(control_rate_hz,
                                                  value_type=float),
                "data_timeout_s": ParameterValue(data_timeout_s,
                                                 value_type=float),
            }],
        ),
    ])
