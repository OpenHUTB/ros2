import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    package_share = get_package_share_directory("px4_mavros2_offboard")
    start_mavros = LaunchConfiguration("start_mavros").perform(context).lower()
    actions = []
    if start_mavros == "true":
        actions.append(
            Node(
                package="mavros",
                executable="mavros_node",
                output="screen",
                parameters=[
                    {
                        "fcu_url": LaunchConfiguration("fcu_url").perform(context),
                        "target_system_id": int(
                            LaunchConfiguration("target_system_id").perform(context)
                        ),
                        "target_component_id": int(
                            LaunchConfiguration("target_component_id").perform(
                                context
                            )
                        ),
                    }
                ],
            )
        )
    actions.append(
        Node(
            package="px4_mavros2_offboard",
            executable="main",
            output="screen",
            parameters=[
                os.path.join(package_share, "config", "offboard.yaml"),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                    .perform(context)
                    .lower()
                    == "true",
                },
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_mavros", default_value="true"),
            DeclareLaunchArgument(
                "fcu_url", default_value="udp://:14540@127.0.0.1:14557"
            ),
            DeclareLaunchArgument("target_system_id", default_value="1"),
            DeclareLaunchArgument("target_component_id", default_value="1"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
