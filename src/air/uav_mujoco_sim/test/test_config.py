#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""config/sim.yaml 必须是合法的 ROS2 参数文件结构。"""
import os

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG_ROOT, "config", "sim.yaml")


def _body_lines():
    with open(CONFIG, encoding="utf-8") as fh:
        return [l.rstrip("\n") for l in fh
                if l.strip() and not l.lstrip().startswith("#")]


def test_has_ros_parameters_node():
    """缺 ``/**/ros__parameters`` 时 rclpy.init 报 "Cannot have a value before
    ros__parameters"，桥接节点直接起不来。"""
    lines = _body_lines()
    assert lines[0].strip() == "/**:"
    assert lines[1] == "  ros__parameters:"


def test_keys_match_declared_parameter_names():
    """参数键名与 bridge_node 的 declare_parameter 一致（含斜杠）。"""
    lines = _body_lines()
    keys = [l.strip().split(":")[0].strip() for l in lines[2:]]
    assert keys, "参数文件为空"
    bad = [k for k in keys if "/" not in k]
    assert not bad, "参数名缺少 '/' 层级：%s" % bad
