import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_directory('humanoid_joint_tools')
    urdf_file = os.path.join(pkg_path, 'urdf', 'simple_humanoid.urdf')
    rviz_file = os.path.join(pkg_path, 'rviz', 'demo.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    use_mock = LaunchConfiguration('use_mock')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_play = LaunchConfiguration('auto_play')
    filename = LaunchConfiguration('filename')
    input_topic = LaunchConfiguration('input_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock', default_value='true',
            description='启动假数据节点（发布到 input_topic）'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='启动 RViz2 可视化'),
        DeclareLaunchArgument(
            'auto_play', default_value='false',
            description='录制文件更新后自动回放一次；默认关闭，改用 /start_play 服务触发'),
        DeclareLaunchArgument(
            'filename', default_value=os.path.expanduser('~/.ros/joint_traj.json'),
            description='轨迹文件路径（录制与回放共用）'),
        DeclareLaunchArgument(
            'input_topic', default_value='/joint_states_raw',
            description='假数据话题，录制节点订阅同一话题；播放节点独占 /joint_states'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
            condition=IfCondition(use_rviz)
        ),
        Node(
            package='humanoid_joint_tools',
            executable='mock',
            name='mock_driver',
            parameters=[{'topic': input_topic}],
            output='screen',
            condition=IfCondition(use_mock)
        ),
        Node(
            package='humanoid_joint_tools',
            executable='record',
            name='joint_recorder',
            parameters=[{'filename': filename, 'input_topic': input_topic}],
            output='screen'
        ),
        Node(
            package='humanoid_joint_tools',
            executable='play',
            name='joint_player',
            parameters=[{'filename': filename,
                         'auto_play': ParameterValue(auto_play, value_type=bool)}],
            output='screen'
        )
    ])
