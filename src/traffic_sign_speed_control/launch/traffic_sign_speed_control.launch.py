# -*- coding: utf-8 -*-
"""一键启动：世界节点 + 检测节点 + 驾驶节点。

用法：
  ros2 launch traffic_sign_speed_control traffic_sign_speed_control.launch.py
可选参数：
  render_mode:=none|file         渲染模式（默认 file，帧写入 frame_dir）
  frame_dir:=/tmp/ts_frames      file 模式帧输出目录
  scene:=default|degraded|dense  预置场景
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    render_mode = LaunchConfiguration("render_mode")
    frame_dir = LaunchConfiguration("frame_dir")
    scene = LaunchConfiguration("scene")

    return LaunchDescription([
        DeclareLaunchArgument("render_mode", default_value="file"),
        DeclareLaunchArgument("frame_dir", default_value="/tmp/ts_frames"),
        DeclareLaunchArgument("scene", default_value="default"),
        Node(package="traffic_sign_speed_control", executable="world_node",
             name="world", output="screen",
             parameters=[{"render_mode": render_mode},
                         {"frame_dir": frame_dir},
                         {"scene": scene}]),
        Node(package="traffic_sign_speed_control", executable="detector_node",
             name="detector", output="screen"),
        Node(package="traffic_sign_speed_control", executable="driver_node",
             name="driver", output="screen"),
    ])
