#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ned_enu_transform 单元测试（不依赖 ROS，任何 Python 3.10+ 可运行）。"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import (  # noqa: E402
    enu_to_ned_position,
    enu_to_ned_quaternion,
    enu_to_ned_yaw,
    ned_to_enu_position,
    ned_to_enu_quaternion,
    ned_to_enu_yaw,
    quaternion_to_yaw_enu,
    wrap_angle,
    yaw_to_quaternion_enu,
)


def test_position_roundtrip():
    """位置往返转换应还原原值。"""
    ned = (10.0, -5.0, -20.0)
    enu = ned_to_enu_position(*ned)
    back = enu_to_ned_position(*enu)
    assert all(abs(a - b) < 1e-12 for a, b in zip(ned, back))


def test_position_signs():
    """NED 的 y(东)、z(下) 转到 ENU 应分别变为 y(北)、z(上) 取反。"""
    # NED: 北10m 东5m 下20m
    enu = ned_to_enu_position(10.0, 5.0, 20.0)
    assert enu == (10.0, -5.0, -20.0)


def test_yaw_signs():
    """NED 偏航 +90°(顺时针右转) 应对应 ENU -90°。"""
    assert abs(ned_to_enu_yaw(math.pi / 2) - (-math.pi / 2)) < 1e-12
    assert abs(enu_to_ned_yaw(-math.pi / 2) - (math.pi / 2)) < 1e-12


def test_yaw_wrap():
    """偏航应归一化到 (-pi, pi]。"""
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-12
    assert abs(wrap_angle(-3 * math.pi) - math.pi) < 1e-12
    assert abs(wrap_angle(0.5)) - 0.5 < 1e-12


def test_quaternion_identity():
    """单位四元数转换后仍为单位四元数。"""
    q = ned_to_enu_quaternion(1.0, 0.0, 0.0, 0.0)
    assert q == (1.0, 0.0, 0.0, 0.0)


def test_quaternion_roundtrip():
    """任意单位四元数往返转换应还原。"""
    q_ned = (0.7071, 0.0, 0.0, 0.7071)
    q_enu = ned_to_enu_quaternion(*q_ned)
    back = enu_to_ned_quaternion(*q_enu)
    assert all(abs(a - b) < 1e-12 for a, b in zip(q_ned, back))


def test_quaternion_norm_preserved():
    """转换不改变四元数模长。"""
    q = (0.5, 0.5, 0.5, 0.5)
    q_enu = ned_to_enu_quaternion(*q)
    n_orig = math.sqrt(sum(c * c for c in q))
    n_new = math.sqrt(sum(c * c for c in q_enu))
    assert abs(n_orig - n_new) < 1e-12


def test_yaw_quaternion_consistency():
    """yaw -> 四元数 -> yaw 应还原（ENU 约定）。"""
    yaw = 1.234
    q = yaw_to_quaternion_enu(yaw)
    assert abs(quaternion_to_yaw_enu(*q) - yaw) < 1e-12


def test_openhutb_takeoff_case():
    """实际案例: OpenHUTB 中无人机在 NED (50, 30, -25) 起飞后应位于
    ENU 北 50m / 东 -30m(即西 30m) / 高 25m。"""
    enu = ned_to_enu_position(50.0, 30.0, -25.0)
    assert abs(enu[0] - 50.0) < 1e-12   # 北向不变
    assert abs(enu[1] + 30.0) < 1e-12   # 东 30 -> 西 30
    assert abs(enu[2] - 25.0) < 1e-12   # 下 -25 -> 上 25


if __name__ == '__main__':
    import pytest
    sys.exit(pytest.main([__file__, '-v']))
