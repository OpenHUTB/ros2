from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bag_path = LaunchConfiguration("bag_path").perform(context)
    save_dir = LaunchConfiguration("save_dir").perform(context)
    max_lidar_points = LaunchConfiguration("max_lidar_points").perform(context)

    parameters = {"bag_path": bag_path, "save_dir": save_dir}
    # 留空时交给节点用自带默认值，避免空字符串转 int 失败
    if max_lidar_points:
        parameters["max_lidar_points"] = int(max_lidar_points)

    node = Node(
        package="bag_blackbox",
        executable="bag_blackbox_main",
        name="bag_analysis_node",
        output="screen",
        parameters=[parameters],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path", default_value="./robot_run",
            description="待分析的 bag 目录或 .bag 文件路径"),
        DeclareLaunchArgument(
            "save_dir", default_value="./output",
            description="图表输出目录"),
        DeclareLaunchArgument(
            "max_lidar_points", default_value="200000",
            description="点云绘图上限，超出后随机降采样"),
        OpaqueFunction(function=launch_setup),
    ])
