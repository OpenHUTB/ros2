"""同时启动发布端与订阅端，QoS 场景由 launch 参数指定。

    ros2 launch joint_qos_latency_eval main.launch.py \
      pub_profile:=best_effort_depth1 sub_profile:=best_effort_depth1 duration:=10
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PACKAGE = 'joint_qos_latency_eval'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pub_profile', default_value='reliable_depth10'),
        DeclareLaunchArgument('sub_profile', default_value='reliable_depth10'),
        DeclareLaunchArgument('rate_hz', default_value='200.0'),
        DeclareLaunchArgument('duration', default_value='10.0'),
        # 默认落到 results/launch.csv（相对当前工作目录），保证 launch 跑完有产出
        DeclareLaunchArgument('csv_path', default_value='results/launch.csv'),
        Node(
            package=PACKAGE,
            executable='joint_qos_pub',
            name='joint_qos_pub',
            output='screen',
            parameters=[{
                'qos_profile': LaunchConfiguration('pub_profile'),
                'rate_hz': LaunchConfiguration('rate_hz'),
                'duration': LaunchConfiguration('duration'),
            }],
        ),
        Node(
            package=PACKAGE,
            executable='joint_qos_sub',
            name='joint_qos_sub',
            output='screen',
            parameters=[{
                'qos_profile': LaunchConfiguration('sub_profile'),
                'duration': LaunchConfiguration('duration'),
                'csv_path': LaunchConfiguration('csv_path'),
                # 应发总条数作为 missing/drop% 的分母
                'expected_msgs': PythonExpression(
                    ['int(', LaunchConfiguration('rate_hz'),
                     ' * ', LaunchConfiguration('duration'), ')']),
            }],
        ),
    ])
