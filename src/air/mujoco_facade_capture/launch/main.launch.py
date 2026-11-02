"""通过 ROS 2 launch 启动传感器和控制话题完整任务节点。"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    package_share = FindPackageShare("mujoco_facade_capture")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output", default_value="/tmp/facade_scan", description="扫描结果目录"
            ),
            DeclareLaunchArgument(
                "style", default_value="grid", description="立面类型"
            ),
            DeclareLaunchArgument(
                "controller", default_value="neural", description="控制器类型"
            ),
            DeclareLaunchArgument(
                "distance", default_value="4.0", description="目标壁距，米"
            ),
            DeclareLaunchArgument(
                "offset", default_value="0.0", description="初始壁距偏移，米"
            ),
            DeclareLaunchArgument(
                "duration", default_value="70.0", description="仿真时长，秒"
            ),
            DeclareLaunchArgument(
                "model",
                default_value=PathJoinSubstitution(
                    [package_share, "models", "facade_controller.json"]
                ),
                description="神经网络权重文件",
            ),
            Node(
                package="mujoco_facade_capture",
                executable="mission",
                name="facade_mission",
                output="screen",
                parameters=[
                    {
                        "output": LaunchConfiguration("output"),
                        "style": LaunchConfiguration("style"),
                        "controller": LaunchConfiguration("controller"),
                        "distance": ParameterValue(
                            LaunchConfiguration("distance"), value_type=float
                        ),
                        "offset": ParameterValue(
                            LaunchConfiguration("offset"), value_type=float
                        ),
                        "duration": ParameterValue(
                            LaunchConfiguration("duration"), value_type=float
                        ),
                        "model": LaunchConfiguration("model"),
                    }
                ],
            ),
        ]
    )
