"""桥接节点感知纯函数单元测试（不依赖 ROS 与 CARLA）。

灯态映射、停止点归一化、最近停车点距离均为纯函数；
地图/交通灯实时查询（sense 的 CARLA 部分）由 live 演示覆盖。
"""

import math
import sys
import types
import unittest


def _ensure_ros_stubs():
    """宿主机无 ROS 时给 rclpy/std_msgs 打桩（缺啥补啥，幂等）。"""
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
    for name, default in (("Float32", 0.0), ("String", "")):
        if not hasattr(msg_mod, name):
            def _mk(default=default):
                class _M:
                    def __init__(self, data=default):
                        self.data = data
                return _M
            setattr(msg_mod, name, _mk())
    sys.modules["std_msgs.msg"] = msg_mod
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs")).msg = msg_mod


_ensure_ros_stubs()

# bridge_node 顶层 import carla——宿主机缺失时打桩（仅纯函数测试）
try:
    import carla  # noqa: F401
except ImportError:
    sys.modules["carla"] = types.SimpleNamespace(
        TrafficLightState=types.SimpleNamespace())

from carla_red_light_warning.bridge_node import (  # noqa: E402
    dist_to_stop, light_state_name, stop_points_xy)


class _Enum:
    """模拟 carla 枚举成员（带 .name）。"""

    def __init__(self, name):
        self.name = name


class _Loc:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _Way:
    """模拟 carla.Waypoint（.location 为 Location）。"""

    def __init__(self, x, y):
        self.location = _Loc(x, y)


class TestLightStateName(unittest.TestCase):

    def test_mapping(self):
        self.assertEqual(light_state_name(_Enum("Red")), "red")
        self.assertEqual(light_state_name(_Enum("Yellow")), "yellow")
        self.assertEqual(light_state_name(_Enum("Green")), "green")
        self.assertEqual(light_state_name(_Enum("Off")), "off")
        self.assertEqual(light_state_name(_Enum("Unknown")), "off")

    def test_invalid_falls_back_to_off(self):
        self.assertEqual(light_state_name(_Enum("Whatever")), "off")
        # 无 .name 属性（异常输入）→ off
        self.assertEqual(light_state_name(object()), "off")


class TestStopPointsXY(unittest.TestCase):

    def test_location_elements(self):
        self.assertEqual(stop_points_xy([_Loc(1, 2), _Loc(3, 4)]),
                         [(1.0, 2.0), (3.0, 4.0)])

    def test_waypoint_elements(self):
        self.assertEqual(stop_points_xy([_Way(5, 6)]), [(5.0, 6.0)])

    def test_empty(self):
        self.assertEqual(stop_points_xy([]), [])


class TestDistToStop(unittest.TestCase):

    def test_nearest_point(self):
        d = dist_to_stop((0, 0), [(10, 0), (3, 4)])   # (3,4) → 5 m
        self.assertAlmostEqual(d, 5.0)

    def test_empty_returns_none(self):
        self.assertIsNone(dist_to_stop((0, 0), []))

    def test_distance_formula(self):
        d = dist_to_stop((1, 1), [(4, 5)])
        self.assertAlmostEqual(d, 5.0)   # 3-4-5 三角形

    def test_sanity_math_hypot(self):
        # 与 math.hypot 一致性
        pts = [(7, 2), (-1, -3)]
        self.assertAlmostEqual(dist_to_stop((2, 2), pts),
                               min(math.hypot(px - 2, py - 2)
                                   for px, py in pts))


if __name__ == "__main__":
    unittest.main()
