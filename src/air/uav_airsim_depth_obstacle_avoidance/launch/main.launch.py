"""launch：启动深度避障节点。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    weights = LaunchConfiguration("weights")
    z_ref = LaunchConfiguration("z_ref")

    return LaunchDescription([
        DeclareLaunchArgument("weights", default_value="", description="训练好的权重文件路径"),
        DeclareLaunchArgument("z_ref", default_value="-5.0", description="基准飞行高度（NED，负值向上）"),
        Node(
            package="uav_airsim_depth_obstacle_avoidance",
            executable="depth_avoid_node",
            name="uav_depth_avoidance",
            output="screen",
            parameters=[{"weights": weights, "z_ref": z_ref}],
        ),
    ])
