"""一键启动 IMU 仿真与两种姿态滤波器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'humanoid_imu_attitude'


def generate_launch_description():
    profile = LaunchConfiguration('profile')
    duration = LaunchConfiguration('duration')
    rate = LaunchConfiguration('rate_hz')
    out_dir = LaunchConfiguration('out_dir')

    arguments = [
        DeclareLaunchArgument('profile', default_value='slow',
                              description='static / slow / fast'),
        DeclareLaunchArgument('duration', default_value='20.0',
                              description='采集时长（秒）'),
        DeclareLaunchArgument('rate_hz', default_value='200.0',
                              description='IMU 发布频率'),
        DeclareLaunchArgument('out_dir', default_value='results',
                              description='CSV 输出目录'),
    ]

    sim = Node(
        package=PACKAGE,
        executable='imu_sim',
        name='imu_sim',
        output='screen',
        parameters=[{
            'profile': profile,
            'duration': ParameterValue(duration, value_type=float),
            'rate_hz': ParameterValue(rate, value_type=float),
            'out_dir': out_dir,
        }],
    )

    filters = []
    for method in ('complementary', 'madgwick'):
        filters.append(Node(
            package=PACKAGE,
            executable='attitude_filter',
            name='attitude_filter_%s' % method,
            output='screen',
            parameters=[{
                'method': method,
                'profile': profile,
                'duration': ParameterValue(duration, value_type=float),
                'out_dir': out_dir,
            }],
        ))

    return LaunchDescription(arguments + [sim] + filters)
