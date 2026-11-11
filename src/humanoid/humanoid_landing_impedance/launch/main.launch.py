"""一键启动落地仿真与评估节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'humanoid_landing_impedance'


def generate_launch_description():
    mode = LaunchConfiguration('controller_mode')
    drop_height = LaunchConfiguration('drop_height')
    duration = LaunchConfiguration('duration')
    out_dir = LaunchConfiguration('out_dir')
    use_gui = LaunchConfiguration('use_gui')

    arguments = [
        DeclareLaunchArgument('controller_mode', default_value='impedance',
                              description='stiff / impedance'),
        DeclareLaunchArgument('drop_height', default_value='0.05',
                              description='跌落高度（米）'),
        DeclareLaunchArgument('duration', default_value='3.0',
                              description='单次仿真时长（秒）'),
        DeclareLaunchArgument('out_dir', default_value='results',
                              description='CSV 输出目录'),
        DeclareLaunchArgument('use_gui', default_value='false',
                              description='是否打开 PyBullet 图形窗口'),
    ]

    sim = Node(
        package=PACKAGE,
        executable='landing_sim',
        name='landing_sim',
        output='screen',
        parameters=[{
            'controller_mode': mode,
            'drop_height': ParameterValue(drop_height, value_type=float),
            'duration': ParameterValue(duration, value_type=float),
            'use_gui': use_gui,
        }],
    )

    evaluator = Node(
        package=PACKAGE,
        executable='landing_evaluate',
        name='landing_evaluate',
        output='screen',
        parameters=[{
            'controller_mode': mode,
            'drop_height': ParameterValue(drop_height, value_type=float),
            'duration': ParameterValue(duration, value_type=float),
            'out_dir': out_dir,
        }],
    )

    return LaunchDescription(arguments + [sim, evaluator])
