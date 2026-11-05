# -*- coding: utf-8 -*-
"""项目统一入口文件（main.py）。

三种用法：
    1) launch 一键启动（推荐，三个节点各起一个进程）：
         ros2 launch carla_yolo_avoidance carla_yolo_avoidance.launch.py

    2) 用三个 console_scripts 分别启动：
         ros2 run carla_yolo_avoidance carla_camera_node
         ros2 run carla_yolo_avoidance yolo_detect_node
         ros2 run carla_yolo_avoidance decision_control_node

    3) 用本入口文件指定角色启动（等价于方式 2）：
         ros2 run carla_yolo_avoidance main --node camera
         ros2 run carla_yolo_avoidance main --node yolo
         ros2 run carla_yolo_avoidance main --node decision

注意：解析 `--node` 后会把它从 sys.argv 里剔除再交给 rclpy，
避免把自定义参数传给 rclpy 的参数解析器。
"""

import os
import sys

from .carla_camera_node import main as camera_main
from .decision_control_node import main as decision_main
from .yolo_detect_node import main as yolo_main

# 角色别名 -> 启动函数
NODE_ENTRIES = {
    'camera': camera_main,
    'carla_camera_node': camera_main,
    'yolo': yolo_main,
    'detect': yolo_main,
    'yolo_detect_node': yolo_main,
    'decision': decision_main,
    'control': decision_main,
    'decision_control_node': decision_main,
}


def _extract_node_role(argv):
    """从参数里取出 --node <role> / --node=<role>，并返回 (role, 去掉该参数后的列表)。"""
    role = None
    remaining = []
    index = 0
    while index < len(argv):
        item = argv[index]
        if item == '--node' and index + 1 < len(argv):
            role = argv[index + 1]
            index += 2
            continue
        if item.startswith('--node='):
            role = item.split('=', 1)[1]
            index += 1
            continue
        remaining.append(item)
        index += 1
    return role, remaining


def main():
    """按角色启动对应节点；未指定时读环境变量 CARLA_YOLO_NODE，默认 camera。"""
    role, remaining = _extract_node_role(sys.argv[1:])
    if role is None:
        role = os.environ.get('CARLA_YOLO_NODE', 'camera')

    entry = NODE_ENTRIES.get(role.lower())
    if entry is None:
        print('未知节点角色 "{}"，可选值：{}'.format(role, sorted(NODE_ENTRIES)))
        raise SystemExit(2)

    # 把剔除自定义参数后的 argv 传下去，rclpy 只看到自己的参数
    sys.argv = [sys.argv[0]] + remaining
    entry()


def main_camera():
    """console_script: carla_camera_node"""
    camera_main()


def main_yolo():
    """console_script: yolo_detect_node"""
    yolo_main()


def main_decision():
    """console_script: decision_control_node"""
    decision_main()


if __name__ == '__main__':
    main()
