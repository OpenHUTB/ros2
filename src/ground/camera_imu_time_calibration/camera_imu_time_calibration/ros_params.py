#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 参数读取辅助：让"照文档敲命令"不会因为参数类型而崩掉。

为什么需要这一层
----------------
``rclpy`` 在 ``declare_parameter`` 时就把参数类型定死了。若节点把 ``offset_ms``
声明成 double，用户按直觉写

    ros2 run camera_imu_time_calibration synthetic_source --ros-args -p offset_ms:=120
    ros2 launch camera_imu_time_calibration main.launch.py offset_ms:=120

覆盖值会被判成 INTEGER，直接抛 ``InvalidParameterTypeException``，节点起不来。
这是"文档里的命令跑不起来"级别的阻塞，用户却很难从报错里看出「加个小数点就行」。

这里的做法是**不带默认值声明**：参数类型保持 NOT_SET（动态类型），任何类型的
覆盖值都能进来，读出后再用 ``float()`` / ``int()`` 强转，缺省值由调用方给。

为什么不用 ``ParameterDescriptor(dynamic_typing=True)``
------------------------------------------------------
那是 Humble 才有的字段，Foxy 上构造 ``ParameterDescriptor`` 就会 ``TypeError``。
NOT_SET + 强转在 Foxy / Humble 上都成立，不引入版本判断，也不用维护两套声明。

代价是 ``ros2 param describe`` 在**没有**覆盖值时会显示类型 ``not set``，
值由代码里的默认值兜底（各节点启动日志会打印最终生效的参数，便于确认）。

本模块不 import rclpy，可以用假节点离线单测（见 ``test/test_params.py``）。
"""

from __future__ import annotations

from typing import Any, Callable


def _declare_and_read(node: Any, name: str, default: Any, cast: Callable) -> Any:
    """以 NOT_SET 类型声明参数，读出后强转；未提供则用默认值。"""
    node.declare_parameter(name)
    value = node.get_parameter(name).value
    return cast(default) if value is None else cast(value)


def number_param(node: Any, name: str, default: float) -> float:
    """读取浮点参数，容忍命令行传入整数（``-p x:=10``）。"""
    return _declare_and_read(node, name, default, float)


def integer_param(node: Any, name: str, default: int) -> int:
    """读取整数参数，容忍命令行传入浮点（``-p x:=10.0``）。"""
    return _declare_and_read(node, name, default, int)


def string_param(node: Any, name: str, default: str) -> str:
    """读取字符串参数（话题名等）。字符串不存在类型歧义，直接用默认值声明。"""
    node.declare_parameter(name, str(default))
    value = node.get_parameter(name).value
    return str(default) if value is None else str(value)


def shutdown_quietly() -> None:
    """安全关闭 rclpy。

    ``ros2 launch`` 或 ``Ctrl-C`` 时，rclpy 的全局 context 可能已经被关过一次，
    再调 ``rclpy.shutdown()`` 会报 ``rcl_shutdown already called on the given
    context``。``try_shutdown()`` 对已关闭的 context 是幂等的；老版本若没有这个
    接口，则先用 ``rclpy.ok()`` 判断。
    """
    import rclpy

    try:
        rclpy.try_shutdown()
    except AttributeError:  # 老版本 rclpy 没有 try_shutdown
        if rclpy.ok():
            rclpy.shutdown()
