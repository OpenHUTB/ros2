"""桥接节点几何测量单元测试（不依赖 ROS 与 CARLA）。

measure() 是纯函数：给定自车位姿与假 waypoint（车道中心线），
验证带符号横向偏移/航向偏差/车道宽的换算。
地图取点（spin_once）依赖 CARLA，由 live 演示覆盖。
"""

import math
import sys
import types
import unittest


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
    sys.modules["std_msgs.msg"] = msg_mod
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs")).msg = msg_mod


_ensure_ros_stubs()

# bridge_node 顶层 import carla——宿主机装了 carla 包则直接用；
# 本测试只调 measure()，若 carla 缺失则给模块对象打桩
try:
    import carla  # noqa: F401,E402
except ImportError:
    sys.modules["carla"] = types.SimpleNamespace(
        LaneType=types.SimpleNamespace(Driving=2))

from carla_lane_departure_warning.bridge_node import LDWBridgeNode  # noqa: E402


class _FakeWP:
    """假 CARLA waypoint：只含 measure() 用到的位姿与车道宽。"""

    def __init__(self, x, y, yaw_deg, width=3.5):
        self.transform = types.SimpleNamespace(
            location=types.SimpleNamespace(x=x, y=y),
            rotation=types.SimpleNamespace(yaw=yaw_deg))
        self.lane_width = width


class _FakeNode(LDWBridgeNode):
    """绕开 rclpy 初始化，仅保留被测的几何逻辑。"""

    def __init__(self):
        pass


class TestMeasure(unittest.TestCase):

    def test_on_centerline(self):
        n = _FakeNode()
        off, herr, w = n.measure((10, 0), 0.0, _FakeWP(10, 0, 0.0))
        self.assertAlmostEqual(off, 0.0)
        self.assertAlmostEqual(herr, 0.0)
        self.assertEqual(w, 3.5)

    def test_left_offset_positive(self):
        n = _FakeNode()
        off, _, _ = n.measure((10, 0.8), 0.0, _FakeWP(10, 0, 0.0))
        self.assertAlmostEqual(off, 0.8)

    def test_right_offset_negative(self):
        n = _FakeNode()
        off, _, _ = n.measure((10, -0.8), 0.0, _FakeWP(10, 0, 0.0))
        self.assertAlmostEqual(off, -0.8)

    def test_rotated_road(self):
        # 道路朝 +y（yaw=90°），自车左偏即 −x 方向
        n = _FakeNode()
        off, _, _ = n.measure((9.2, 5), 90.0, _FakeWP(10, 5, 90.0))
        self.assertAlmostEqual(off, 0.8, delta=1e-6)

    def test_heading_error_deg(self):
        n = _FakeNode()
        # ego yaw 95°，道路 90° → 左偏 5°
        _, herr, _ = n.measure((10, 0), 95.0, _FakeWP(10, 0, 90.0))
        self.assertAlmostEqual(herr, math.radians(5.0), delta=1e-9)

    def test_lane_width_passthrough(self):
        n = _FakeNode()
        _, _, w = n.measure((10, 0), 0.0, _FakeWP(10, 0, 0.0, width=2.8))
        self.assertEqual(w, 2.8)


if __name__ == "__main__":
    unittest.main()
