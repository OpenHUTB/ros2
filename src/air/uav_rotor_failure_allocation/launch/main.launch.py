"""启动四旋翼单桨失效推力分配节点."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 与节点参数同名, 默认值保持一致; failed_rotor 用 -1 表示四桨完好.
ARGUMENTS = (('mass', '2.0'), ('arm_length', '0.25'), ('torque_ratio', '0.016'),
             ('max_thrust', '0.0'), ('failed_rotor', '-1'))


def generate_launch_description():
    """返回模块默认启动描述, 机型参数与失效桨可在命令行覆盖."""
    declared = [DeclareLaunchArgument(name, default_value=value) for name, value in ARGUMENTS]
    parameters = [{'mass': ParameterValue(LaunchConfiguration('mass'), value_type=float),
                   'arm_length': ParameterValue(LaunchConfiguration('arm_length'), value_type=float),
                   'torque_ratio': ParameterValue(LaunchConfiguration('torque_ratio'), value_type=float),
                   'max_thrust': ParameterValue(LaunchConfiguration('max_thrust'), value_type=float),
                   'failed_rotor': ParameterValue(LaunchConfiguration('failed_rotor'), value_type=int)}]
    return LaunchDescription(declared + [
        Node(package='uav_rotor_failure_allocation', executable='node',
             output='screen', parameters=parameters),
    ])
