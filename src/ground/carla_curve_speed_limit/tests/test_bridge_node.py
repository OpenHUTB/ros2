"""桥接节点逻辑单元测试（不依赖 ROS 与 CARLA）。

测目标速度换算与控制缓存；地图取点（_front_points）依赖 CARLA，
由 live 演示覆盖。
"""

import sys
import types
import unittest

import numpy as np


def _ensure_ros_stubs():
    """宿主机无 ROS 时给 rclpy/std_msgs 打桩（幂等）。"""
    try:
        import rclpy  # noqa: F401
        return
    except ImportError:
        pass
    if "rclpy" not in sys.modules:
        class _Node:
            pass

        rclpy_mod = types.ModuleType("rclpy")
        node_mod = types.ModuleType("rclpy.node")
        node_mod.Node = _Node
        rclpy_mod.node = node_mod
        sys.modules["rclpy"] = rclpy_mod
        sys.modules["rclpy.node"] = node_mod

    msg_mod = sys.modules.setdefault(
        "std_msgs.msg", types.ModuleType("std_msgs.msg"))
    if not hasattr(msg_mod, "Float32"):
        class Float32:
            def __init__(self, data=0.0):
                self.data = data
        msg_mod.Float32 = Float32
    if not hasattr(msg_mod, "Float32MultiArray"):
        class Float32MultiArray:
            def __init__(self, data=None):
                self.data = data or []
        msg_mod.Float32MultiArray = Float32MultiArray
    sys.modules["std_msgs.msg"] = msg_mod
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs")).msg = msg_mod


_ensure_ros_stubs()

import math  # noqa: E402

from carla_curve_speed_limit.bridge_node import CSLBridgeNode  # noqa: E402


class _FakeNode(CSLBridgeNode):
    """绕开 rclpy 初始化，仅保留被测逻辑与参数默认值。"""

    def __init__(self, **params):
        cfg = {"look_ahead": 30.0, "a_lat_max": 2.5, "v_min": 1.5,
               "valid_max": 30.0}
        cfg.update(params)
        from carla_curve_speed_limit.curve_speed_core import CSLParams
        self.p = CSLParams(**{k: cfg[k] for k in
                              ("look_ahead", "a_lat_max", "v_min", "valid_max")})
        self._control = [0.0, 0.0]


def arc_points(radius, n=40, total_angle=math.pi / 2):
    """半径 radius 的左转圆弧路点。"""
    cy = radius
    pts = [(0.0, 0.0)]
    for i in range(1, n):
        ang = total_angle * i / n
        pts.append((radius * math.sin(ang), cy - radius * math.cos(ang)))
    return pts


class TestControlCache(unittest.TestCase):

    def test_on_control_stores_pair(self):
        n = _FakeNode()
        n._on_control(type("M", (), {"data": [0.7, 0.3]})())
        self.assertEqual(n._control, [0.7, 0.3])

    def test_default_no_control(self):
        n = _FakeNode()
        self.assertEqual(n._control, [0.0, 0.0])


class TestTargetSpeed(unittest.TestCase):

    def test_straight_cruise_cap(self):
        pts = [(i * 1.0, 0.0) for i in range(20)]
        n = _FakeNode()
        # 直道限速=valid_max(30)，但目标速度取 min(巡航, 限速) = 巡航 8.0
        self.assertEqual(n._target_speed(pts), 8.0)

    def test_arc_limited(self):
        n = _FakeNode()
        pts = arc_points(10.0)   # R=10, a=2.5 → 5.0 m/s
        self.assertAlmostEqual(n._target_speed(pts), 5.0, delta=0.15)

    def test_v_min_floor(self):
        n = _FakeNode()
        pts = arc_points(1.0)    # 极急弯理论 1.58 → 实际由 v_min 兜底
        t = n._target_speed(pts)
        self.assertGreaterEqual(t, n.p.v_min)


class TestCurveRadius(unittest.TestCase):

    def test_straight_sentinel(self):
        n = _FakeNode()
        pts = [(i * 1.0, 0.0) for i in range(10)]
        self.assertEqual(n._curve_radius(pts), 999.0)

    def test_arc_radius(self):
        n = _FakeNode()
        pts = arc_points(10.0, n=40)
        r = n._curve_radius(pts)
        self.assertAlmostEqual(r, 10.0, delta=0.5)

    def test_too_few_points(self):
        n = _FakeNode()
        self.assertEqual(n._curve_radius([(0, 0), (1, 0)]), 999.0)


if __name__ == "__main__":
    unittest.main()
