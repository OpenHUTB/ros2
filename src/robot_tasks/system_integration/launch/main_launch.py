"""系统集成 launch：一次拉起键盘、相机、导航、策略四个模块，经集成层汇流。

    ros2 launch system_integration main_launch.py                    # 默认：策略驱动
    ros2 launch system_integration main_launch.py keyboard:=true     # 外加键盘手动
    ros2 launch system_integration main_launch.py nav:=true          # 外加地图与规划

四个模块的输出都先改名到各自通道（/cmd_vel_manual、/cmd_vel_auto、/plan_nav），
再由集成节点仲裁成唯一的 /cmd_vel，避免同一话题出现两个发布者。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('policy', default_value='true',
                             description='启动任务4 策略节点与二维仿真'),
        DeclareLaunchArgument('keyboard', default_value='false',
                             description='启动任务1 键盘遥操作'),
        DeclareLaunchArgument('camera', default_value='false',
                             description='启动任务2 模拟相机'),
        DeclareLaunchArgument('nav', default_value='false',
                             description='启动任务3 地图与规划器'),
        DeclareLaunchArgument('timeout', default_value='0.5'),
        DeclareLaunchArgument('max_linear', default_value='1.0'),
        DeclareLaunchArgument('max_angular', default_value='1.5'),
    ]

    manager = Node(
        package='system_integration', executable='main_manager',
        name='integration_manager', output='screen',
        parameters=[{
            'timeout': ParameterValue(LaunchConfiguration('timeout'),
                                      value_type=float),
            'max_linear': ParameterValue(LaunchConfiguration('max_linear'),
                                         value_type=float),
            'max_angular': ParameterValue(LaunchConfiguration('max_angular'),
                                          value_type=float),
        }],
    )

    # 任务4：仿真提供 /odom，策略节点输出到 /cmd_vel_auto
    sim = Node(
        package='policy_learning', executable='main_sim', name='sim_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('policy')),
    )
    policy = Node(
        package='policy_learning', executable='main_policy', name='policy_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('policy')),
        remappings=[('/cmd_vel', '/cmd_vel_auto'), ('/plan', '/plan_auto')],
    )

    # 任务1：键盘输出改名到 /cmd_vel_manual
    teleop = Node(
        package='teleop_control', executable='main_teleop',
        name='teleop_control', output='screen',
        condition=IfCondition(LaunchConfiguration('keyboard')),
        remappings=[('/cmd_vel', '/cmd_vel_manual')],
    )

    # 任务2：模拟相机
    camera_node = Node(
        package='sensor_perception', executable='main_sensor',
        name='sensor_perception', output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    # 任务3：地图与 A* 规划，路径改名到 /plan_nav，避免和策略的 /plan 冲突
    # 任务4 的仿真也会发布 /map（12 m×12 m），任务3 是 10 m×10 m，两者不能共用一个话题
    nav_map = Node(
        package='navigation_slam', executable='main_map', name='map_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('nav')),
        remappings=[('/map', '/map_nav')],
    )
    nav_planner = Node(
        package='navigation_slam', executable='main_planner',
        name='path_planner', output='screen',
        condition=IfCondition(LaunchConfiguration('nav')),
        remappings=[('/map', '/map_nav'), ('/plan', '/plan_nav')],
    )

    return LaunchDescription(args + [
        manager,
        TimerAction(period=1.0, actions=[sim]),
        TimerAction(period=2.0, actions=[policy]),
        TimerAction(period=1.0, actions=[teleop]),
        TimerAction(period=1.0, actions=[camera_node]),
        TimerAction(period=1.0, actions=[nav_map]),
        TimerAction(period=2.0, actions=[nav_planner]),
    ])
