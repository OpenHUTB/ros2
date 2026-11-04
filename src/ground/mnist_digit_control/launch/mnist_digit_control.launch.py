# ROS2 launch：一键启动 turtlesim + 数字检测节点 + 数字指令控制节点
# 用法：
#   ros2 launch mnist_digit_control mnist_digit_control.launch.py
#   ros2 launch mnist_digit_control mnist_digit_control.launch.py \
#       source:=video source_path:=/abs/path/scene_paper.mp4 display:=true
import glob
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SRC_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _find_share() -> str:
    """优先使用 colcon 安装后的 share 目录，否则回退源码目录（直接在源码树运行时）。"""
    try:
        return get_package_share_directory("mnist_digit_control")
    except PackageNotFoundError:
        return SRC_PKG_DIR


def _default(rel: str, fallback: str) -> str:
    share = _find_share()
    p = os.path.join(share, rel)
    if os.path.exists(p):
        return p
    return fallback


def generate_launch_description():
    share = _find_share()
    default_model = _default(
        os.path.join("models", "mnist_cnn.pth"),
        os.path.join(SRC_PKG_DIR, "models", "mnist_cnn.pth"),
    )
    videos = sorted(glob.glob(os.path.join(share, "videos", "scene_*.mp4"))
                    or glob.glob(os.path.join(SRC_PKG_DIR, "videos", "scene_*.mp4")))
    # 默认优先 paper 场景（背景干净、识别最稳定），否则取排序第一个
    preferred = [v for v in videos if os.path.basename(v) == "scene_paper.mp4"]
    default_video = (preferred or videos)[0] if videos else ""

    source = LaunchConfiguration("source")
    source_path = LaunchConfiguration("source_path")
    model_path = LaunchConfiguration("model_path")
    display = LaunchConfiguration("display")

    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="video",
                              description="视频源类型：video | webcam | images"),
        DeclareLaunchArgument("source_path", default_value=default_video,
                              description="视频文件路径（webcam 模式忽略）"),
        DeclareLaunchArgument("model_path", default_value=default_model,
                              description="CNN 权重路径"),
        DeclareLaunchArgument("display", default_value="true",
                              description="是否显示 OpenCV 可视化窗口"),

        Node(package="turtlesim", executable="turtlesim_node",
             name="turtlesim", output="screen"),

        Node(package="mnist_digit_control", executable="detector",
             name="digit_detector", output="screen",
             parameters=[{"source": source,
                          "source_path": source_path,
                          "model_path": model_path,
                          "display": display,
                          "conf_threshold": 0.7,
                          "rate": 10.0}]),

        Node(package="mnist_digit_control", executable="commander",
             name="digit_command", output="screen",
             parameters=[{"stable_frames": 6,
                          "conf_threshold": 0.7,
                          "cooldown": 0.8}]),
    ])
