"""一键启动膝关节阶跃仿真与评估节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'humanoid_knee_impedance'


def generate_launch_description():
    kp = LaunchConfiguration('knee_kp')
    kd = LaunchConfiguration('knee_kd')
    step_target = LaunchConfiguration('step_target')
    duration = LaunchConfiguration('duration')
    out_dir = LaunchConfiguration('out_dir')
    use_gui = LaunchConfiguration('use_gui')

    arguments = [
        DeclareLaunchArgument('knee_kp', default_value='10.0',
                              description='膝关节刚度（N·m/rad）'),
        DeclareLaunchArgument('knee_kd', default_value='0.50',
                              description='膝关节阻尼（N·m·s/rad）'),
        DeclareLaunchArgument('step_target', default_value='0.6',
                              description='阶跃目标角（rad）'),
        DeclareLaunchArgument('duration', default_value='2.0',
                              description='阶跃后仿真时长（秒）'),
        DeclareLaunchArgument('out_dir', default_value='results',
                              description='CSV 输出目录'),
        DeclareLaunchArgument('use_gui', default_value='false',
                              description='是否打开 PyBullet 图形窗口'),
    ]

    sim = Node(
        package=PACKAGE,
        executable='knee_sim',
        name='knee_sim',
        output='screen',
        parameters=[{
            'knee_kp': ParameterValue(kp, value_type=float),
            'knee_kd': ParameterValue(kd, value_type=float),
            'step_target': ParameterValue(step_target, value_type=float),
            'duration': ParameterValue(duration, value_type=float),
            'use_gui': ParameterValue(use_gui, value_type=bool),
            'out_dir': out_dir,
        }],
    )

    return LaunchDescription(arguments + [sim])
