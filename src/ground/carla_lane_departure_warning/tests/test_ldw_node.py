"""LDW 预警节点逻辑单元测试（不依赖 ROS 与 CARLA）。

用桩替代 rclpy/std_msgs，直接驱动 _step 的状态发布与事件去重语义。
"""

import sys
import types
import unittest


def _ensure_ros_stubs():
    """宿主机无 ROS 时给 rclpy/std_msgs 打桩（缺啥补啥，幂等）。"""
    try:
        import rclpy  # noqa: F401
    except ImportError:
        if "rclpy" not in sys.modules:
            class _Node:
                pass

            rclpy_mod = types.ModuleType("rclpy")
            node_mod = types.ModuleType("rclpy.node")
            node_mod.Node = _Node
            rclpy_mod.node = node_mod

            def _fake(*a, **k):
                raise RuntimeError("ROS 未安装：本测试只验证预警逻辑，不应触发生命周期")

            rclpy_mod.init = _fake
            rclpy_mod.spin = _fake
            sys.modules["rclpy"] = rclpy_mod
            sys.modules["rclpy.node"] = node_mod

    msg_mod = sys.modules.setdefault(
        "std_msgs.msg", types.ModuleType("std_msgs.msg"))
    if not hasattr(msg_mod, "Float32"):
        class Float32:
            def __init__(self, data=0.0):
                self.data = data
        msg_mod.Float32 = Float32
    if not hasattr(msg_mod, "String"):
        class String:
            def __init__(self, data=""):
                self.data = data
        msg_mod.String = String
    sys.modules["std_msgs.msg"] = msg_mod
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs")).msg = msg_mod


_ensure_ros_stubs()

from carla_lane_departure_warning.ldw_node import LDWAlertNode  # noqa: E402
from carla_lane_departure_warning.lane_departure_core import (  # noqa: E402
    LDWController, LDWParams)


class _FakeNode(LDWAlertNode):
    """绕开 rclpy 初始化，手工补齐被测成员，捕获发布负载。"""

    def __init__(self, **overrides):
        cfg = {"warn_enter": 0.5, "warn_exit": 0.3, "depart_enter": 0.9,
               "depart_exit": 0.65, "period": 0.05, "warmup_ticks": 2}
        cfg.update(overrides)
        self.p = LDWParams(**{k: cfg[k] for k in (
            "warn_enter", "warn_exit", "depart_enter", "depart_exit")})
        self.period = cfg["period"]
        self.ctrl = LDWController(self.p, warmup_ticks=cfg["warmup_ticks"])
        self._offset = 0.0
        self._last_state = self.ctrl._state
        self.states = []
        self.events = []

        class _Pub:
            def __init__(self, sink):
                self._sink = sink

            def publish(self, msg):
                self._sink.append(msg.data)

        self.pub_state = _Pub(self.states)
        self.pub_event = _Pub(self.events)

        class _Log:
            def info(self, *a, **k):
                pass

        self.get_logger = lambda: _Log()

    def feed(self, offset):
        self._offset = float(offset)
        self._step()


class TestStatePublishing(unittest.TestCase):

    def test_state_published_every_tick(self):
        n = _FakeNode()
        n.feed(0.0)
        n.feed(0.0)
        self.assertEqual(len(n.states), 2)
        self.assertEqual(n.states[0], "normal")

    def test_offset_subscription_updates(self):
        n = _FakeNode()
        n.feed(0.0)   # 热身 2 拍
        n.feed(0.0)
        n.feed(0.8)   # deviation 区
        self.assertEqual(n.states[-1], "deviation")
        self.assertEqual(n.events, ["deviation_start_left"])


class TestEventDedup(unittest.TestCase):

    def test_transition_event_emitted_once(self):
        n = _FakeNode()
        n.feed(0.8)   # normal → deviation
        n.feed(0.8)   # 保持，无新事件
        n.feed(0.8)
        self.assertEqual(n.events, ["deviation_start_left"])
        self.assertEqual(n.states[-1], "deviation")

    def test_full_cycle_event_chain(self):
        n = _FakeNode()
        # 前两拍 0.0 为热身；后续 0.7→1.1→回中 触发完整事件链
        for o in [0.0, 0.0, 0.7, 1.1, 1.1, 0.7, 0.2, 0.0]:
            n.feed(o)
        self.assertEqual(n.events, ["deviation_start_left",
                                    "departure_start_left",
                                    "departure_end",
                                    "deviation_clear"])
        self.assertEqual(n.states[-1], "normal")

    def test_right_side_event(self):
        n = _FakeNode()
        n.feed(0.0)   # 热身 2 拍
        n.feed(0.0)
        n.feed(-0.7)
        self.assertEqual(n.events[-1], "deviation_start_right")

    def test_warmup_suppresses_cold_start_false_alarm(self):
        """起步首拍大偏移（spawn 投影不稳）不得触发假报警。"""
        n = _FakeNode()
        n.feed(1.65)   # 热身拍：实测冷启动偏移 1.648 m，须被抑制
        n.feed(0.15)
        self.assertEqual(n.events, [])
        self.assertEqual(n.states, ["normal", "normal"])
        n.feed(0.8)    # 热身结束后的真实偏离正常报警
        self.assertEqual(n.events, ["deviation_start_left"])

    def test_no_event_while_staying(self):
        n = _FakeNode()
        n.feed(1.0)    # normal 一拍跨入 departure（1.0 ≥ 0.9），只发一次事件
        n.feed(1.0)
        n.feed(1.0)
        self.assertEqual(n.events, ["departure_start_left"])
        self.assertEqual(n.states[-1], "departure")


if __name__ == "__main__":
    unittest.main()
