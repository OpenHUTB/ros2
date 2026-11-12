"""启动均值滤波节点，可通过 launch 参数覆盖话题与窗口大小."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """构建 mean_filter 节点的启动描述."""
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    window_size = LaunchConfiguration('window_size')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic', default_value='/sensor/raw',
            description='订阅的原始传感器话题'),
        DeclareLaunchArgument(
            'output_topic', default_value='/sensor/filtered',
            description='滤波结果的发布话题'),
        DeclareLaunchArgument(
            'window_size', default_value='5',
            description='滑动窗口长度，必须是正整数'),
        Node(
            package='mean_filter_pkg',
            executable='mean_filter',
            name='mean_filter_node',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
                'window_size': ParameterValue(
                    window_size, value_type=int),
            }],
        ),
    ])
