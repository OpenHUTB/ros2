from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    publish_rate = LaunchConfiguration('publish_rate')
    sequence = LaunchConfiguration('sequence')
    enable_plot = LaunchConfiguration('enable_plot')
    output_dir = LaunchConfiguration('output_dir')
    export_every = LaunchConfiguration('export_every')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_rate', default_value='1.0',
            description='速度指令发布频率（Hz）'),
        DeclareLaunchArgument(
            'sequence', default_value='forward,turn_left,hover',
            description='指令序列，逗号分隔：forward/backward/turn_left/turn_right/hover'),
        DeclareLaunchArgument(
            'enable_plot', default_value='true',
            description='是否打开 matplotlib 实时轨迹窗口'),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description='轨迹记录导出目录，留空则使用当前工作目录'),
        DeclareLaunchArgument(
            'export_every', default_value='0',
            description='每 N 个采样点导出一次记录，0 表示只在退出时导出'),
        Node(
            package='drone_trajectory_logger',
            executable='main',
            name='trajectory_publisher',
            output='screen',
            parameters=[{
                'publish_rate': ParameterValue(publish_rate, value_type=float),
                'sequence': sequence,
            }],
        ),
        Node(
            package='drone_trajectory_logger',
            executable='analyzer',
            name='trajectory_analyzer',
            output='screen',
            parameters=[{
                'enable_plot': ParameterValue(enable_plot, value_type=bool),
                'output_dir': output_dir,
                'export_every': ParameterValue(export_every, value_type=int),
            }],
        ),
    ])
