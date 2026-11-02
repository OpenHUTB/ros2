"""ROS 2 launch 入口：一条命令启动无人机深度避障节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    params = os.path.join(get_package_share_directory("uav_mujoco_depth_avoidance"),
                          "config", "params.yaml")
    return LaunchDescription([
        Node(
            package="uav_mujoco_depth_avoidance",
            executable="uav_depth_avoidance_node",
            name="uav_mujoco_depth_avoidance",
            output="screen",
            parameters=[ParameterFile(params, allow_substs=True)],
        )
    ])
