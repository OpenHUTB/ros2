"""MuJoCo默认运行，支持传入AirSim主机与路线参数。"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription


def generate_launch_description():
    defaults = {
        "backend": ("mujoco", str),
        "style": ("brick", str),
        "output": ("/tmp/town10_cruise", str),
        "duration": ("100.0", float),
        "speed": ("0.8", float),
        "host": ("127.0.0.1", str),
        "port": ("41451", int),
        "vehicle": ("", str),
        "camera": ("0", str),
        "route": ("relative", str),
        "takeoff": ("false", bool),
        "gif": ("false", bool),
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, (value, _) in defaults.items()
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=kind)
        for name, (_, kind) in defaults.items()
    }
    return LaunchDescription(
        declarations
        + [
            Node(
                package="town10_facade_cruise_capture",
                executable="mission",
                output="screen",
                parameters=[parameters],
            )
        ]
    )
