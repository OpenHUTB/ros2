"""RLW 预警节点逻辑单元测试（不依赖 ROS 与 CARLA）。

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

from carla_red_light_warning.rlw_node import RLWAlertNode  # noqa: E402
from carla_red_light_warning.red_light_core import (  # noqa: E402
    RLWController, RLWParams)


class _FakeNode(RLWAlertNode):
    """绕开 rclpy 初始化，手工补齐被测成员，捕获发布负载。"""

    def __init__(self, **overrides):
        cfg = {"a_comfort": 2.5, "warn_enter": 30.0, "warn_exit": 35.0,
               "brake_margin": 1.2, "period": 0.05, "warmup_ticks": 2}
        cfg.update(overrides)
        self.p = RLWParams(**{k: cfg[k] for k in (
            "a_comfort", "warn_enter", "warn_exit", "brake_margin")})
        self.period = cfg["period"]
        self.ctrl = RLWController(self.p, warmup_ticks=cfg["warmup_ticks"])
        self._light = "off"
        self._dist = 1.0e4
        self._speed = 0.0
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

    def feed(self, light, dist, speed):
        self._light = light
        self._dist = float(dist)
        self._speed = float(speed)
        self._step()

    def feed_light_invalid(self, bad):
        """验证 _on_light 对非法灯态的防御。"""
        class _M:
            data = bad
        self._on_light(_M())
        return self._light


class TestStatePublishing(unittest.TestCase):

    def test_state_published_every_tick(self):
        n = _FakeNode()
        n.feed("green", 100.0, 8.0)
        n.feed("green", 100.0, 8.0)
        self.assertEqual(len(n.states), 2)
        self.assertEqual(n.states[0], "normal")

    def test_subscription_updates(self):
        n = _FakeNode()
        n.feed("green", 100.0, 8.0)   # 热身拍 1
        n.feed("green", 100.0, 8.0)   # 热身拍 2
        n.feed("red", 20.0, 8.0)      # approaching 区
        self.assertEqual(n.states[-1], "approaching")
        self.assertEqual(n.events, ["approach_start"])


class TestEventDedup(unittest.TestCase):

    def test_transition_event_emitted_once(self):
        n = _FakeNode()
        n.feed("green", 100.0, 8.0)
        n.feed("green", 100.0, 8.0)
        n.feed("red", 25.0, 8.0)     # approach_start
        n.feed("red", 25.0, 8.0)     # 保持，无新事件
        n.feed("red", 25.0, 8.0)
        self.assertEqual(n.events, ["approach_start"])
        self.assertEqual(n.states[-1], "approaching")

    def test_full_cycle_event_chain(self):
        n = _FakeNode()
        for light, d in [("green", 100), ("green", 100),   # 热身
                         ("red", 25), ("red", 25),         # approaching
                         ("red", 10), ("red", 10),         # violation
                         ("green", 10), ("green", 10)]:    # 恢复
            n.feed(light, d, 8.0)
        self.assertEqual(n.events, ["approach_start", "violation_start",
                                    "violation_clear"])
        self.assertEqual(n.states[-1], "normal")

    def test_no_event_while_staying(self):
        n = _FakeNode()
        n.feed("green", 100, 8.0)
        n.feed("green", 100, 8.0)
        n.feed("red", 10.0, 8.0)    # 一拍跨入 violation（d_stop=12.8>10）
        n.feed("red", 10.0, 8.0)
        n.feed("red", 10.0, 8.0)
        self.assertEqual(n.events, ["violation_start"])
        self.assertEqual(n.states[-1], "violation_risk")

    def test_warmup_suppresses_cold_start(self):
        n = _FakeNode()
        n.feed("red", 5.0, 10.0)    # 热身拍：本应直接 violation
        n.feed("red", 5.0, 10.0)
        self.assertEqual(n.events, [])
        self.assertEqual(n.states, ["normal", "normal"])

    def test_invalid_light_rejected(self):
        n = _FakeNode()
        self.assertEqual(n.feed_light_invalid("blue"), "off")
        self.assertEqual(n.feed_light_invalid("red"), "red")   # 合法值照常更新


if __name__ == "__main__":
    unittest.main()
