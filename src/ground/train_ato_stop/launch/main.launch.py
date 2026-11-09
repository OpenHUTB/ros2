"""启动 ATO 回放节点与 RViz2。

scenario 参数决定跑哪个场景：
  flat / restriction / grade / late_brake

speed_scale 是速度到高度的折算系数（m per km/h），默认 4.0，
160 km/h 会画到线路以上 640 m 的位置。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('train_ato_stop')

    scenario = DeclareLaunchArgument(
        'scenario', default_value='flat',
        description='flat / restriction / grade / late_brake')
    speed_scale = DeclareLaunchArgument(
        'speed_scale', default_value='4.0',
        description='速度折算到高度用的系数，单位 m/(km/h)')
    playback_rate = DeclareLaunchArgument(
        'playback_rate', default_value='1.0',
        description='回放倍速')

    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'train_ato_stop.rviz'])
    params = PathJoinSubstitution([pkg_share, 'config', 'train_ato.yaml'])

    node = Node(
        package='train_ato_stop',
        executable='train_ato_node',
        name='train_ato_stop',
        output='screen',
        parameters=[
            params,
            {
                # 参数必须显式声明类型，否则启动时按字符串传进去会抛
                # InvalidParameterTypeException
                'scenario': ParameterValue(LaunchConfiguration('scenario'), value_type=str),
                'speed_scale': ParameterValue(LaunchConfiguration('speed_scale'), value_type=float),
                'playback_rate': ParameterValue(LaunchConfiguration('playback_rate'), value_type=float),
            },
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([scenario, speed_scale, playback_rate, node, rviz])
