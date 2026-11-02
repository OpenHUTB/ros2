"""默认单次处理随包图像；mode:=live 时连接巡航相机话题。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="samples"),
        DeclareLaunchArgument("input_topic", default_value="/town10/image_raw"),
        DeclareLaunchArgument("output", default_value="town10_detections"),
        DeclareLaunchArgument("confidence", default_value="0.25"),
        Node(package="town10_facade_defect_detection", executable="facade_detect", output="screen",
             parameters=[{"mode": LaunchConfiguration("mode"), "input_topic": LaunchConfiguration("input_topic"),
                          "output": LaunchConfiguration("output"),
                          "confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float)}])])
