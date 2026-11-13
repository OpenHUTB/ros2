#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""figure8_sim 的离线测试：不需要 ROS、不需要 RViz、不需要 pytest。

用法::

    python3 test/test_figure8_sim_local.py

脚本会桩掉 rospy / visualization_msgs（仅 import 用），检查：

1. 轨迹：原点、最右点、有界性、x/y 周期；
2. 速度：与中心差分一致（这条用例曾抓到 y 导数的系数错误）；
3. 航向：等于切向、处处有限、四元数为单位四元数；
4. Marker：frame/type/action/位置/朝向/尺寸与轨迹一致、位姿随时间变化。
"""

from __future__ import annotations

import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCRIPTS = os.path.join(ROOT, "scripts")
sys.path.insert(0, SCRIPTS)

PASS, FAIL = 0, 0


def check(cond, label):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  [PASS] %s" % label)
    else:
        FAIL += 1
        print("  [FAIL] %s" % label)


def _install_stubs():
    class Time(object):
        def __init__(self, secs=0.0):
            self._secs = float(secs)

        @staticmethod
        def now():
            return Time(0.0)

        def to_sec(self):
            return self._secs

        def __sub__(self, other):
            return Time(self._secs - other._secs)

    rospy = types.ModuleType("rospy")
    rospy.Time = Time
    rospy.init_node = lambda *a, **k: None
    rospy.is_shutdown = lambda: True
    rospy.spin = lambda *a, **k: None

    class _Pub(object):
        def __init__(self, *a, **k):
            self.sent = []

        def publish(self, msg):
            self.sent.append(msg)

    class _Rate(object):
        def __init__(self, hz):
            self.hz = hz

        def sleep(self):
            pass

    rospy.Publisher = _Pub
    rospy.Rate = _Rate
    rospy.ROSInterruptException = type("ROSInterruptException", (Exception,), {})

    vmsg = types.ModuleType("visualization_msgs")
    msg = types.ModuleType("visualization_msgs.msg")

    class _Vec(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class _Quat(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0
            self.w = 1.0

    class _Color(object):
        def __init__(self):
            self.r = self.g = self.b = self.a = 0.0

    class _Header(object):
        def __init__(self):
            self.frame_id = ""
            self.stamp = None

    class _Pose(object):
        def __init__(self):
            self.position = _Vec()
            self.orientation = _Quat()

    class Marker(object):
        ADD = 0
        CUBE = 1

        def __init__(self):
            self.header = _Header()
            self.type = None
            self.action = None
            self.pose = _Pose()
            self.scale = _Vec()
            self.color = _Color()

    msg.Marker = Marker
    vmsg.msg = msg

    sys.modules["rospy"] = rospy
    sys.modules["visualization_msgs"] = vmsg
    sys.modules["visualization_msgs.msg"] = msg


_install_stubs()

import figure8  # noqa: E402
import main as sim  # noqa: E402


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def test_curve():
    print("== 轨迹 ==")
    x, y = figure8.position(0.0)
    check(close(x, 0.0) and close(y, 0.0), "t=0 在原点")

    x, y = figure8.position(math.pi / 2)
    check(close(x, 2.0) and close(y, 0.0, 1e-12), "t=pi/2 在最右点 (2, 0)")

    bounded = True
    for k in range(400):
        x, y = figure8.position(k * 0.05)
        bounded = bounded and (-2.0 - 1e-9 <= x <= 2.0 + 1e-9) \
            and (-1.0 - 1e-9 <= y <= 1.0 + 1e-9)
    check(bounded, "曲线有界：|x|<=2，|y|<=1")

    periodic = all(close(figure8.position(t + 2 * math.pi)[0],
                         figure8.position(t)[0], 1e-9) for t in (0.0, 0.7, 2.3))
    check(periodic, "x 的周期为 2*pi")
    periodic = all(close(figure8.position(t + math.pi)[1],
                         figure8.position(t)[1], 1e-9) for t in (0.0, 0.4, 1.9))
    check(periodic, "y 的周期为 pi")


def test_velocity():
    print("== 速度 ==")
    h = 1e-6
    ok = True
    for t in (0.0, 0.6, 1.7, 3.0):
        x1, y1 = figure8.position(t + h)
        x0, y0 = figure8.position(t - h)
        vx, vy = figure8.velocity(t)
        ok = ok and close(vx, (x1 - x0) / (2 * h), 1e-6) \
            and close(vy, (y1 - y0) / (2 * h), 1e-6)
    check(ok, "速度与中心差分一致（y 导数系数回归）")


def test_heading_and_quaternion():
    print("== 航向与四元数 ==")
    ok = True
    for t in (0.0, 0.5, 1.2, 2.5):
        vx, vy = figure8.velocity(t)
        ok = ok and close(math.atan2(vy, vx), figure8.heading(t), 1e-12)
    check(ok, "航向等于切向方向")

    finite = True
    for k in range(200):
        theta = figure8.heading(k * 0.03)
        finite = finite and math.isfinite(theta) and -math.pi <= theta <= math.pi
    check(finite, "航向处处有限且在 [-pi, pi]")

    unit = True
    for t in (0.0, 1.0, 2.2):
        qx, qy, qz, qw = figure8.quaternion(t)
        unit = unit and qx == 0.0 and qy == 0.0 and close(qz * qz + qw * qw, 1.0, 1e-12)
    check(unit, "四元数为绕 z 的单位四元数")


def test_marker():
    print("== Marker ==")
    marker = sim.build_marker(0.0)
    check(marker.header.frame_id == "map", "frame_id = map")
    check(marker.type == marker.CUBE and marker.action == marker.ADD,
          "type=CUBE, action=ADD")
    check(close(marker.pose.position.z, sim.MARKER_Z), "z 高度正确")
    check(close(marker.scale.x, sim.SCALE[0]) and close(marker.scale.y, sim.SCALE[1])
          and close(marker.scale.z, sim.SCALE[2]), "尺寸与常量一致")

    t = 1.1
    marker = sim.build_marker(t)
    x, y = figure8.position(t)
    _qx, _qy, qz, qw = figure8.quaternion(t)
    check(close(marker.pose.position.x, x) and close(marker.pose.position.y, y),
          "位置与轨迹一致")
    check(close(marker.pose.orientation.z, qz) and close(marker.pose.orientation.w, qw),
          "朝向与轨迹一致")

    a = sim.build_marker(0.0).pose.position
    b = sim.build_marker(1.0).pose.position
    check((a.x, a.y) != (b.x, b.y), "位姿随时间变化")


def main():
    test_curve()
    test_velocity()
    test_heading_and_quaternion()
    test_marker()
    print("-" * 50)
    print("通过 %d / 失败 %d" % (PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
