import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("uwb_range_odom_fusion")
    config = os.path.join(share, "config", "demo.yaml")
    rviz_config = os.path.join(share, "rviz", "uwb_fusion.rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "duration",
                default_value="0.0",
                description="simulation length in seconds, 0 means loop forever",
            ),
            DeclareLaunchArgument(
                "seed", default_value="0", description="random seed of the simulator"
            ),
            DeclareLaunchArgument(
                "rviz", default_value="false", description="start rviz2 with the demo view"
            ),
            Node(
                package="uwb_range_odom_fusion",
                executable="uwb_sim",
                name="uwb_sim",
                output="screen",
                parameters=[
                    config,
                    {
                        "duration_s": ParameterValue(
                            LaunchConfiguration("duration"), value_type=float
                        ),
                        "seed": ParameterValue(
                            LaunchConfiguration("seed"), value_type=int
                        ),
                    },
                ],
            ),
            Node(
                package="uwb_range_odom_fusion",
                executable="uwb_fusion",
                name="uwb_fusion",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
