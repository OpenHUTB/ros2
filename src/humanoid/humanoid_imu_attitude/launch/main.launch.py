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
    alpha = LaunchConfiguration('alpha')
    beta = LaunchConfiguration('beta')

    arguments = [
        DeclareLaunchArgument('profile', default_value='slow',
                              description='static / slow / fast'),
        DeclareLaunchArgument('duration', default_value='20.0',
                              description='采集时长（秒）'),
        DeclareLaunchArgument('rate_hz', default_value='200.0',
                              description='IMU 发布频率'),
        DeclareLaunchArgument('out_dir', default_value='results',
                              description='CSV 输出目录'),
        DeclareLaunchArgument('alpha', default_value='0.98',
                              description='互补滤波陀螺仪权重，越大越信任陀螺仪'),
        DeclareLaunchArgument('beta', default_value='0.05',
                              description='Madgwick 梯度下降增益，越大校正越快'),
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
                'alpha': ParameterValue(alpha, value_type=float) if method == 'complementary' else 0.0,
                'beta': ParameterValue(beta, value_type=float) if method == 'madgwick' else 0.0,
            }],
        ))

    return LaunchDescription(arguments + [sim] + filters)
