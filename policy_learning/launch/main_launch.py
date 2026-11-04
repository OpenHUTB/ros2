"""启动：二维仿真节点 + 策略节点。

    ros2 launch policy_learning main_launch.py
    ros2 launch policy_learning main_launch.py rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("policy_learning"), "config", "plan.rviz"])

    args = [
        DeclareLaunchArgument("rviz", default_value="true",
                             description="是否同时打开 rviz"),
        DeclareLaunchArgument("model", default_value="",
                             description="策略权重路径，留空用包内默认"),
        DeclareLaunchArgument("start_x", default_value="0.85"),
        DeclareLaunchArgument("start_y", default_value="3.05"),
    ]

    sim = Node(
        package="policy_learning", executable="main_sim",
        name="sim_node", output="screen",
        parameters=[{
            "start_x": ParameterValue(LaunchConfiguration("start_x"),
                                      value_type=float),
            "start_y": ParameterValue(LaunchConfiguration("start_y"),
                                      value_type=float),
            "rate_hz": 20.0,
        }],
    )

    policy = Node(
        package="policy_learning", executable="main_policy",
        name="policy_node", output="screen",
        parameters=[{
            "model": LaunchConfiguration("model"),
            "rate_hz": ParameterValue("10.0", value_type=float),
        }],
    )

    rviz = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(args + [
        sim,
        TimerAction(period=1.0, actions=[policy]),
        TimerAction(period=2.0, actions=[rviz]),
    ])
