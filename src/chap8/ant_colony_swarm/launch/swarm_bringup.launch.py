"""启动视觉检测与编队控制节点。

前置条件：需要先运行能发布 /camera/image_raw 的相机仿真（本 launch 不启动
Gazebo），详见 docs/ant/README.md「前置条件」一节。
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vision_node = Node(
        package="ant_colony_swarm",
        executable="vision_detector",
        output="screen",
    )
    ctrl_node = Node(
        package="ant_colony_swarm",
        executable="swarm_control",
        output="screen",
    )
    return LaunchDescription([
        vision_node,
        ctrl_node,
    ])
