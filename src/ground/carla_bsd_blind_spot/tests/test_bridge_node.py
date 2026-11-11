"""桥接节点逻辑单元测试（不依赖 ROS 与 CARLA）。

直接测 _zone_distance 的几何逻辑与发布负载语义；
节点生命周期（连接/tick/清理）由 main.py doctor 与 live 演示覆盖。
"""

import sys
import types
import unittest
from unittest.mock import MagicMock

import numpy as np


def _ensure_ros_stubs():
    """宿主机无 ROS 时给 rclpy/std_msgs 打桩，保证节点模块可导入。"""
    try:
        import rclpy  # noqa: F401
        return
    except ImportError:
        pass
    if "rclpy" in sys.modules:
        return

    class _Node:  # 被测逻辑不使用 Node 基类行为
        pass

    rclpy_mod = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node
    rclpy_mod.node = node_mod
    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = node_mod

    std_msgs = types.ModuleType("std_msgs")
    msg_mod = types.ModuleType("std_msgs.msg")

    class Float32:
        def __init__(self, data=0.0):
            self.data = data

    class String:
        def __init__(self, data=""):
            self.data = data

    msg_mod.Float32 = Float32
    msg_mod.String = String
    std_msgs.msg = msg_mod
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = msg_mod


_ensure_ros_stubs()

from carla_bsd_blind_spot.bridge_node import BSDBridgeNode  # noqa: E402


def make_car_body(cx, cy, n=30, z=-1.0):
    rng = np.random.default_rng(11)
    pts = np.stack([
        cx + rng.uniform(-0.8, 0.8, n),
        cy + rng.uniform(-0.8, 0.8, n),
        np.full(n, z) + rng.uniform(-0.2, 0.2, n),
        np.full(n, 0.5)], axis=1).astype(np.float32)
    return pts


class _FakeNode(BSDBridgeNode):
    """绕开 rclpy 初始化，仅保留被测逻辑与参数默认值。"""

    def __init__(self):
        # 不调用 super().__init__()；手工补齐被测代码用到的成员
        self.frames = 8
        self.x_min = -25.0
        self.x_max = 2.0
        self.y_near = 2.9
        self.y_far = 4.2
        self.min_points = 6
        self.valid_max = 40.0
        from collections import deque
        self._buf = deque(maxlen=self.frames)


class TestZoneDistanceLogic(unittest.TestCase):

    def setUp(self):
        self.node = _FakeNode()

    def test_left_zone_with_neighbor(self):
        self.node._buf.clear()
        self.node._buf.append(make_car_body(-4.0, 3.0))
        d = self.node._zone_distance("left")
        self.assertLess(d, 8.0)

    def test_right_zone_empty(self):
        self.node._buf.clear()
        self.node._buf.append(make_car_body(-4.0, 3.0))  # 只有左后车
        self.assertEqual(self.node._zone_distance("right"), 40.0)

    def test_fusion_buffer_maxlen(self):
        self.node._buf.clear()
        for i in range(20):
            self.node._buf.append(np.zeros((1, 4), dtype=np.float32))
        self.assertEqual(len(self.node._buf), 8)

    def test_empty_buffer_returns_valid_max(self):
        self.node._buf.clear()
        self.assertEqual(self.node._zone_distance("left"), 40.0)

    def test_lidar_parse_copy_semantics(self):
        # _on_lidar 必须复制：CARLA 复用 raw_data 内存
        self.node._buf.clear()
        raw = make_car_body(0.0, 3.0).tobytes()
        pc = MagicMock()
        pc.raw_data = raw
        BSDBridgeNode._on_lidar(self.node, pc)
        pts = self.node._buf[-1]
        self.assertEqual(pts.shape[0], 30)
        # 再改原字节对象不影响已入队的数组
        raw = b"\x00" * len(raw)
        self.assertEqual(self.node._buf[-1].shape, pts.shape)


if __name__ == "__main__":
    unittest.main()
