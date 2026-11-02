"""启动无人机定高指令限幅与超时保持节点."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 与节点参数同名，默认值保持一致，可在 ros2 launch 时用 name:=value 覆盖。
ARGUMENTS = (('minimum', '0.4'), ('maximum', '3.0'), ('rate', '0.5'),
             ('timeout', '0.6'), ('initial', '1.5'))


def generate_launch_description():
    """返回模块的默认启动描述，约束参数可在命令行覆盖."""
    declared = [DeclareLaunchArgument(name, default_value=value) for name, value in ARGUMENTS]
    parameters = [{name: ParameterValue(LaunchConfiguration(name), value_type=float)}
                  for name, _ in ARGUMENTS]
    return LaunchDescription(declared + [
        Node(package='uav_altitude_command_guard', executable='node',
             output='screen', parameters=parameters),
    ])
