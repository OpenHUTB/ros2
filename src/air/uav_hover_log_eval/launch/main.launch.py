"""启动无人机定高日志误差与恢复时间评估节点."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """声明日志与统计参数，并传入摘要发布节点."""
    defaults = {
        'disturbance_end': '3.0', 'disturbance_window': '1.0',
        'tolerance': '0.15', 'hold_s': '0.5', 'max_gap_s': '0.05',
    }
    log_path = PathJoinSubstitution([
        FindPackageShare('uav_hover_log_eval'), 'data', 'gust_ppo_101.csv'])
    arguments = [DeclareLaunchArgument('log_path', default_value=log_path)]
    arguments.extend(DeclareLaunchArgument(name, default_value=value)
                     for name, value in defaults.items())
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in defaults
    }
    parameters['log_path'] = ParameterValue(
        LaunchConfiguration('log_path'), value_type=str)
    return LaunchDescription(arguments + [
        Node(package='uav_hover_log_eval', executable='node',
             output='screen', parameters=[parameters])])
