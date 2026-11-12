"""启动过渡走廊节点。

用法::

    ros2 launch uav_tiltrotor_transition_corridor transition.launch.py
    ros2 launch uav_tiltrotor_transition_corridor transition.launch.py corridor_altitude:=4000.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("altitude_ref", default_value="30.0", description="目标海拔高度（m）"),
        DeclareLaunchArgument("airspeed_ref", default_value="0.0", description="目标空速（m/s）"),
        DeclareLaunchArgument("corridor_altitude", default_value="0.0",
                              description="走廊建表的密度高度（m），须与机场一致"),
    ]
    return LaunchDescription(
        args
        + [
            Node(
                package="uav_tiltrotor_transition_corridor",
                executable="transition_corridor_node",
                name="transition_corridor_node",
                output="screen",
                parameters=[{
                    "altitude_ref": LaunchConfiguration("altitude_ref"),
                    "airspeed_ref": LaunchConfiguration("airspeed_ref"),
                    "corridor_altitude": LaunchConfiguration("corridor_altitude"),
                }],
            )
        ]
    )
