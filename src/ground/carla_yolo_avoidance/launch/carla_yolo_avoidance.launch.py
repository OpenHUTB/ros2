# -*- coding: utf-8 -*-
"""一键启动三个节点（+ 可选 RViz）。

用法：
    ros2 launch carla_yolo_avoidance carla_yolo_avoidance.launch.py
    ros2 launch carla_yolo_avoidance carla_yolo_avoidance.launch.py use_rviz:=false
    ros2 launch carla_yolo_avoidance carla_yolo_avoidance.launch.py params_file:=/path/to/my_params.yaml

参数从哪来：
    CARLA 的 IP / 端口、YOLOv8 权重这些都写在
    config/params.yaml 里，由本 launch 文件加载并下发给三个节点。
    换机器、换端口只需要改 YAML，不用改任何 Python 源码。

启动内容：
    1. carla_camera_node    —— 连接 CARLA、生成车辆、挂相机、发原图、执行控制
    2. yolo_detect_node     —— YOLOv8 推理，发检测框 + 距离
    3. decision_control_node—— 按距离阈值发油门/刹车
    4. static_transform_publisher —— 发布 map -> carla_camera_link，供 RViz 显示检测框
    5. rviz2（可用 use_rviz:=false 关闭）—— 显示原图与检测框
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'carla_yolo_avoidance'


def generate_launch_description():
    """构造 LaunchDescription。"""
    package_share = get_package_share_directory(PACKAGE_NAME)
    rviz_config = os.path.join(package_share, 'rviz', 'carla_yolo.rviz')
    # colcon build 后 config/params.yaml 会被安装到这里
    default_params = os.path.join(package_share, 'config', 'params.yaml')

    # ---------------- 可配置项 ----------------
    # 算法阈值仍然写死在 config.py 里，这里只暴露"文件位置"和"开关"
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='ROS 参数文件路径（CARLA 的 IP / 端口、YOLO 权重都在里面）',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否同时启动 RViz2 可视化',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='RViz2 配置文件路径',
    )

    # 参数文件按节点名分节（/carla_camera_node、/yolo_detect_node ...），
    # 三个节点都加载同一份文件，ROS 2 会各自取属于自己名字的那一节。
    params_file = LaunchConfiguration('params_file')

    # ---------------- 节点①：CARLA 相机 ----------------
    carla_camera_node = Node(
        package=PACKAGE_NAME,
        executable='carla_camera_node',
        name='carla_camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    # ---------------- 节点②：YOLOv8 检测 ----------------
    yolo_detect_node = Node(
        package=PACKAGE_NAME,
        executable='yolo_detect_node',
        name='yolo_detect_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    # ---------------- 节点③：避障决策 ----------------
    decision_control_node = Node(
        package=PACKAGE_NAME,
        executable='decision_control_node',
        name='decision_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    # ---------------- 静态 TF：让 RViz 有 map 坐标系可显示 ----------------
    # 说明：检测框标记以 map 为参考系，x 表示前方距离、y 表示横向偏移，
    #       所以这里只需要一个固定不变的 map -> carla_camera_link 变换。
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='carla_camera_tf',
        output='log',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'carla_camera_link',
        ],
    )

    # ---------------- RViz2 ----------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_rviz,
        declare_rviz_config,
        static_tf_node,
        carla_camera_node,
        yolo_detect_node,
        decision_control_node,
        rviz_node,
    ])
