from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def value(name, kind):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    arguments = [
        DeclareLaunchArgument(
            'publish_rate', default_value='1.0',
            description='速度指令发布频率（Hz），必须为正数'),
        DeclareLaunchArgument(
            'sequence', default_value='forward,turn_left,hover',
            description='指令序列，逗号分隔：forward/backward/turn_left/turn_right/hover'),
        DeclareLaunchArgument(
            'linear_speed', default_value='1.0',
            description='前进/后退指令的速度系数（m/s）'),
        DeclareLaunchArgument(
            'angular_speed', default_value='1.0',
            description='左转/右转指令的角速度系数（rad/s）'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/drone/cmd_vel',
            description='速度指令话题（发布与订阅共用）'),
        DeclareLaunchArgument(
            'state_topic', default_value='/drone/flight_state',
            description='识别结果（std_msgs/String）发布话题'),
        DeclareLaunchArgument(
            'step_length', default_value='0.5',
            description='每个采样步的位移（米）'),
        DeclareLaunchArgument(
            'turn_step', default_value='0.2',
            description='每个采样步的转角（弧度）'),
        DeclareLaunchArgument(
            'linear_deadband', default_value='0.01',
            description='线速度死区（|v| 小于该值视为静止）'),
        DeclareLaunchArgument(
            'angular_deadband', default_value='0.01',
            description='角速度死区（|w| 小于该值视为不转）'),
        DeclareLaunchArgument(
            'enable_plot', default_value='true',
            description='是否打开 matplotlib 实时轨迹窗口（无显示器时设 false）'),
        DeclareLaunchArgument(
            'export_on_shutdown', default_value='true',
            description='退出时是否自动导出记录'),
        DeclareLaunchArgument(
            'export_every', default_value='0',
            description='每 N 个采样点导出一次记录，0 表示只在退出时导出'),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description='轨迹记录导出目录，留空则使用当前工作目录'),
        DeclareLaunchArgument(
            'csv_name', default_value='drone_trajectory.csv',
            description='逐点 CSV 文件名'),
        DeclareLaunchArgument(
            'json_name', default_value='drone_trajectory_summary.json',
            description='汇总 JSON 文件名'),
    ]

    return LaunchDescription(arguments + [
        Node(
            package='drone_trajectory_logger',
            executable='main',
            name='trajectory_publisher',
            output='screen',
            parameters=[{
                'publish_rate': value('publish_rate', float),
                'sequence': LaunchConfiguration('sequence'),
                'linear_speed': value('linear_speed', float),
                'angular_speed': value('angular_speed', float),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }],
        ),
        Node(
            package='drone_trajectory_logger',
            executable='analyzer',
            name='trajectory_analyzer',
            output='screen',
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'state_topic': LaunchConfiguration('state_topic'),
                'step_length': value('step_length', float),
                'turn_step': value('turn_step', float),
                'linear_deadband': value('linear_deadband', float),
                'angular_deadband': value('angular_deadband', float),
                'enable_plot': value('enable_plot', bool),
                'export_on_shutdown': value('export_on_shutdown', bool),
                'export_every': value('export_every', int),
                'output_dir': LaunchConfiguration('output_dir'),
                'csv_name': LaunchConfiguration('csv_name'),
                'json_name': LaunchConfiguration('json_name'),
            }],
        ),
    ])
