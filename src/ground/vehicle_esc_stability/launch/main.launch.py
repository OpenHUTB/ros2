"""启动 ESC 回放节点与 RViz2。

scenario 参数决定回放哪个工况：
  lane_change / sine_dwell / mu_split / wet_lane_change

节点的参数同时写在 config/vehicle_esc.yaml 里，这里的启动参数会覆盖文件，
方便直接 ros2 launch vehicle_esc_stability main.launch.py scenario:=mu_split。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('vehicle_esc_stability')

    scenario = DeclareLaunchArgument(
        'scenario', default_value='lane_change',
        description='lane_change / sine_dwell / mu_split / wet_lane_change')
    playback_rate = DeclareLaunchArgument(
        'playback_rate', default_value='1.0',
        description='回放倍速')
    frame_step = DeclareLaunchArgument(
        'frame_step', default_value='4',
        description='每个周期跨过的仿真帧数（单帧 0.05 s）')
    rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否同时打开 RViz2')

    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'vehicle_esc_stability.rviz'])
    params = PathJoinSubstitution([pkg_share, 'config', 'vehicle_esc.yaml'])

    node = Node(
        package='vehicle_esc_stability',
        executable='vehicle_esc_node',
        name='vehicle_esc_stability',
        output='screen',
        parameters=[
            params,
            {
                # 参数必须显式声明类型，否则启动时按字符串传进去会抛
                # InvalidParameterTypeException
                'scenario': ParameterValue(LaunchConfiguration('scenario'), value_type=str),
                'playback_rate': ParameterValue(
                    LaunchConfiguration('playback_rate'), value_type=float),
                'frame_step': ParameterValue(LaunchConfiguration('frame_step'), value_type=int),
            },
        ],
    )

    view = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([scenario, playback_rate, frame_step, rviz, node, view])
