"""一键启动：避碰节点 + RViz2。

    ros2 launch usv_colreg_avoidance main.launch.py
    ros2 launch usv_colreg_avoidance main.launch.py scenario:=head_on
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('usv_colreg_avoidance')

    scenario = LaunchConfiguration('scenario')
    d_safe = LaunchConfiguration('d_safe')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='crossing',
            description='会遇场景：head_on / crossing / overtaking / multi'),
        DeclareLaunchArgument(
            'd_safe', default_value='400.0',
            description='安全会遇距离 D_safe（米）'),

        Node(
            package='usv_colreg_avoidance',
            executable='usv_colreg_node',
            name='usv_colreg_avoidance',
            output='screen',
            parameters=[{
                'scenario': scenario,
                # 必须显式声明成 double，否则 launch 传进去的是字符串，
                # 节点声明 double 参数时会抛 InvalidParameterTypeException
                'd_safe': ParameterValue(d_safe, value_type=float),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', share + '/rviz/usv_colreg_avoidance.rviz'],
            output='screen',
        ),
    ])
