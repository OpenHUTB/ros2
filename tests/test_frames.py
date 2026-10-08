#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""坐标换算单元测试（不需要 ROS / 仿真器，可直接 pytest 运行）.

验证 NED→ENU 换算的物理正确性：
  * AirSim 单位姿态（机头朝北）在 ENU 下偏航角应为 +90°（因为 ENU 以“东”为 0°）
  * AirSim 偏航 90°（机头朝东）在 ENU 下偏航角应为 0°
  * 位置/速度按 x↔y 交换、z 取反
  * 四元数 ↔ 旋转矩阵往返一致
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "src", "air", "carlair_ros_bridge", "scripts"))

from sim_client import (  # noqa: E402
    C_NED2ENU, airsim_quat_to_ros_quat, enu_to_ned, matrix_to_quat_xyzw,
    ned_to_enu, quat_wxyz_to_matrix, yaw_from_quat_xyzw,
)


def _quat_yaw_ned(yaw_rad):
    """NED 下绕 z 轴（向下为正）旋转 yaw 的四元数 (w, x, y, z)."""
    return [math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)]


def test_position_conversion():
    assert np.allclose(ned_to_enu([1.0, 2.0, -3.0]), [2.0, 1.0, 3.0])
    assert np.allclose(enu_to_ned([2.0, 1.0, 3.0]), [1.0, 2.0, -3.0])
    assert np.allclose(ned_to_enu(enu_to_ned([0.5, -1.0, 2.0])), [0.5, -1.0, 2.0])


def test_c_matrix_is_rotation():
    assert np.isclose(np.linalg.det(C_NED2ENU), 1.0)
    assert np.allclose(C_NED2ENU @ C_NED2ENU.T, np.eye(3))


def test_identity_attitude_faces_north_in_enu():
    """AirSim 单位姿态 = 机头朝北；ENU 下“北”是 +y，故偏航角应为 +90°."""
    q_ros = airsim_quat_to_ros_quat([1.0, 0.0, 0.0, 0.0])
    assert np.isclose(math.degrees(yaw_from_quat_xyzw(q_ros)), 90.0, atol=1e-6)


def test_yaw_90_ned_faces_east_in_enu():
    """AirSim 偏航 90° = 机头朝东；ENU 下“东”是 +x，故偏航角应为 0°."""
    q_ros = airsim_quat_to_ros_quat(_quat_yaw_ned(math.radians(90.0)))
    assert np.isclose(math.degrees(yaw_from_quat_xyzw(q_ros)), 0.0, atol=1e-6)


def test_yaw_180_ned_faces_south_in_enu():
    q_ros = airsim_quat_to_ros_quat(_quat_yaw_ned(math.radians(180.0)))
    assert np.isclose(abs(math.degrees(yaw_from_quat_xyzw(q_ros))), 90.0, atol=1e-6)


def test_quat_matrix_roundtrip():
    for yaw in (0.0, 30.0, 120.0, -75.0):
        q = _quat_yaw_ned(math.radians(yaw))
        R = quat_wxyz_to_matrix(q)
        x, y, z, w = matrix_to_quat_xyzw(R)
        R2 = quat_wxyz_to_matrix([w, x, y, z])
        assert np.allclose(R, R2, atol=1e-9)


def test_body_axes_mapping():
    """单位姿态下：机体 x（北）-> ENU +y，机体 y（东）-> ENU +x，机体 z（下）-> ENU -z."""
    R_enu = C_NED2ENU @ quat_wxyz_to_matrix([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(R_enu[:, 0], [0.0, 1.0, 0.0])
    assert np.allclose(R_enu[:, 1], [1.0, 0.0, 0.0])
    assert np.allclose(R_enu[:, 2], [0.0, 0.0, -1.0])


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("  [PASS] %s" % name)
            except AssertionError as exc:
                failures += 1
                print("  [FAIL] %s -> %s" % (name, exc))
    print("坐标换算测试: %s" % ("全部通过" if failures == 0 else "%d 项失败" % failures))
    sys.exit(1 if failures else 0)
