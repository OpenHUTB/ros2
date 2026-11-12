"""启动 G1 柔顺控制仿真。

    ros2 launch humanoid_g1_compliance main.launch.py
    ros2 launch humanoid_g1_compliance main.launch.py stiffness:="[55,250,250,55,55]"
    ros2 launch humanoid_g1_compliance main.launch.py teleop:=false

参数：
  stiffness    5 通道刚度 [K_lin_la, K_lin_ra, K_lin_to, K_ang_la, K_ang_ra]
  base_height  骨盆目标高度 m（0.56~0.78）
  direct_start true 时以满刚度直接进入 POLICY（从训练站姿起步最快）
  teleop       true 时额外启动终端调刚度节点
  device       cpu / cuda:0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stiffness = LaunchConfiguration('stiffness')
    base_height = LaunchConfiguration('base_height')
    direct_start = LaunchConfiguration('direct_start')
    teleop = LaunchConfiguration('teleop')
    device = LaunchConfiguration('device')

    sim_node = Node(
        package='humanoid_g1_compliance',
        executable='sim_node',
        name='g1_compliance_sim',
        output='screen',
        parameters=[{
            'stiffness': stiffness,
            'base_height': base_height,
            'direct_start': direct_start,
            'device': device,
        }],
    )

    stiffness_node = Node(
        package='humanoid_g1_compliance',
        executable='stiffness_node',
        name='g1_compliance_stiffness',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(teleop),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'stiffness', default_value='[250.0, 250.0, 250.0, 55.0, 55.0]',
            description='5 通道刚度 K_lin_la/K_lin_ra/K_lin_to/K_ang_la/K_ang_ra'),
        DeclareLaunchArgument(
            'base_height', default_value='0.74', description='骨盆目标高度 m'),
        DeclareLaunchArgument(
            'direct_start', default_value='true',
            description='true 时跳过软启动，直接满刚度进入 POLICY'),
        DeclareLaunchArgument(
            'teleop', default_value='true', description='是否启动终端调刚度节点'),
        DeclareLaunchArgument(
            'device', default_value='cpu', description='推理设备'),
        sim_node,
        stiffness_node,
    ])
