"""launch 启动入口：一次拉起规划节点与 RViz2。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('parking_path_planner'), 'rviz', 'parking_path_planner.rviz'])

    arguments = [
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否同时启动 RViz2'),
        DeclareLaunchArgument('slot_center_x', default_value='0.0'),
        DeclareLaunchArgument('slot_center_y', default_value='2.75'),
        DeclareLaunchArgument('slot_heading_deg', default_value='0.0'),
        DeclareLaunchArgument('start_x', default_value='-7.0'),
        DeclareLaunchArgument('start_y', default_value='-4.2'),
        DeclareLaunchArgument('start_yaw_deg', default_value='0.0'),
    ]

    # LaunchConfiguration 取出来是字符串，必须转成 float，否则节点里声明为 double 的
    # 参数会报 InvalidParameterTypeException。
    planner_node = Node(
        package='parking_path_planner',
        executable='parking_path_planner',
        name='parking_path_planner',
        output='screen',
        parameters=[{
            'slot_center_x': ParameterValue(
                LaunchConfiguration('slot_center_x'), value_type=float),
            'slot_center_y': ParameterValue(
                LaunchConfiguration('slot_center_y'), value_type=float),
            'slot_heading_deg': ParameterValue(
                LaunchConfiguration('slot_heading_deg'), value_type=float),
            'start_x': ParameterValue(
                LaunchConfiguration('start_x'), value_type=float),
            'start_y': ParameterValue(
                LaunchConfiguration('start_y'), value_type=float),
            'start_yaw_deg': ParameterValue(
                LaunchConfiguration('start_yaw_deg'), value_type=float),
            'auto_plan': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )

    return LaunchDescription(arguments + [planner_node, rviz_node])
