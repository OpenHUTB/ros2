# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""uav_slam_nav 总入口。

一条命令拉起整套“感知 -> 建图 -> 规划 -> 控制”链路，也可以只起其中一段：

    ros2 run uav_slam_nav main --role all          # 全部四个节点
    ros2 run uav_slam_nav main --role source       # 只起 AirSim 数据源
    ros2 run uav_slam_nav main --role mapping      # 只起建图
    ros2 run uav_slam_nav main --role planner      # 只起规划
    ros2 run uav_slam_nav main --role controller   # 只起控制

``--role all`` 时四个节点共享一个 ``MultiThreadedExecutor``，任何节点的回调被
阻塞（比如 AirSim 的 RPC 等待）都不会拖住其它节点。
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from uav_slam_nav.airsim_source import AirSimSource
from uav_slam_nav.controller import Controller
from uav_slam_nav.mapping import Mapping
from uav_slam_nav.planner import Planner

ROLE_CHOICES = ('all', 'source', 'mapping', 'planner', 'controller')

BUILDERS = {
    'source': AirSimSource,
    'mapping': Mapping,
    'planner': Planner,
    'controller': Controller,
}


def _split_role(argv):
    """从命令行里取出 ``--role``，返回 (role, 去掉 role 后的参数列表)。

    ROS2 自己的参数（``--ros-args`` 之后的部分）原样保留，交给 ``rclpy.init``。
    """
    role = 'all'
    rest = []
    index = 0
    while index < len(argv):
        item = argv[index]
        if item in ('--role', '-r') and index + 1 < len(argv):
            role = argv[index + 1]
            index += 2
            continue
        if item.startswith('--role='):
            role = item.split('=', 1)[1]
            index += 1
            continue
        rest.append(item)
        index += 1
    return role, rest


def _print_usage():
    """打印用法。"""
    print('用法：main [--role {all,source,mapping,planner,controller}] '
          '[--ros-args ...]')
    print('  all         依次启动 source / mapping / planner / controller（默认）')
    print('  source      只起 AirSim 数据源节点')
    print('  mapping     只起占据栅格建图节点')
    print('  planner     只起 A* 规划节点')
    print('  controller  只起纯追踪控制节点')


def main(argv=None):
    """程序入口。"""
    args = list(sys.argv if argv is None else argv)
    role, ros_args = _split_role(args)
    if role not in ROLE_CHOICES:
        print(f'未知的 --role：{role}')
        _print_usage()
        return 2

    rclpy.init(args=ros_args)
    nodes = []
    executor = MultiThreadedExecutor()
    try:
        names = ROLE_CHOICES[1:] if role == 'all' else (role,)
        for name in names:
            node = BUILDERS[name]()
            nodes.append(node)
            executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for node in nodes:
            if isinstance(node, Mapping) and bool(node.get_parameter('save_map').value):
                try:
                    node.save_map()
                except Exception as exc:  # noqa: BLE001 - 退出阶段不抛异常
                    node.get_logger().warn(f'保存地图失败：{exc}')
            shutdown = getattr(node, 'shutdown', None)
            if callable(shutdown):
                try:
                    shutdown()
                except Exception:  # noqa: BLE001
                    pass
        executor.shutdown()
        for node in nodes:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001 - 可能已被 executor 销毁
                pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
