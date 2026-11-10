from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    node = Node(
        package="bag_blackbox",
        executable="bag_blackbox_main",
        name="bag_analysis_node",
        output="screen",
        parameters=[{
            "bag_path": LaunchConfiguration("bag_path").perform(context),
            "save_dir": LaunchConfiguration("save_dir").perform(context),
        }],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path", default_value="./robot_run.bag",
            description="待分析的 bag 文件路径"),
        DeclareLaunchArgument(
            "save_dir", default_value="./docs/img",
            description="图表输出目录"),
        OpaqueFunction(function=launch_setup),
    ])
