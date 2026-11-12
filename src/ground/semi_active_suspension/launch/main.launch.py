"""启动悬架回放节点与 RViz2。

scenario 参数决定回放哪个工况：
  speed_bump / brake_dive / cornering_roll / iso_rough

节点的参数同时写在 config/semi_active_suspension.yaml 里，这里的启动参数会覆盖
文件，方便直接 ros2 launch semi_active_suspension main.launch.py scenario:=iso_rough。

rviz 默认关：无显示环境（CI、容器、WSL 无 X）里 RViz2 找不到 Ogre 的渲染库会直接
exit 127，而节点本身是好的。要看画面自己加 rviz:=true。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('semi_active_suspension')

    scenario = DeclareLaunchArgument(
        'scenario', default_value='speed_bump',
        description='speed_bump / brake_dive / cornering_roll / iso_rough')
    playback_rate = DeclareLaunchArgument(
        'playback_rate', default_value='1.0',
        description='回放倍速')
    frame_step = DeclareLaunchArgument(
        'frame_step', default_value='15',
        description='每个周期跨过的采样帧数（单帧 0.002 s）')
    height_gain = DeclareLaunchArgument(
        'height_gain', default_value='4.0',
        description='垂向位移放大倍数，1.0 为真实比例')
    rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='是否同时打开 RViz2；无显示环境请保持 false')

    rviz_config = PathJoinSubstitution(
        [pkg_share, 'rviz', 'semi_active_suspension.rviz'])
    params = PathJoinSubstitution([pkg_share, 'config', 'semi_active_suspension.yaml'])

    node = Node(
        package='semi_active_suspension',
        executable='semi_active_suspension_node',
        name='semi_active_suspension',
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
                'height_gain': ParameterValue(
                    LaunchConfiguration('height_gain'), value_type=float),
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

    return LaunchDescription([scenario, playback_rate, frame_step, height_gain, rviz, node, view])
