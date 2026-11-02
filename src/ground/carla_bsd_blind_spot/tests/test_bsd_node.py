"""BSD 预警节点逻辑单元测试（不依赖 ROS 与 CARLA）。

用桩替代 rclpy/std_msgs，直接驱动 _step 的决策-发布语义：
分级（normal/approach/warn）、事件去重（跃迁才发）、左右独立。
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
                raise RuntimeError("ROS 未安装：本测试只验证决策逻辑，不应触发生命周期")

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

import json  # noqa: E402

from carla_bsd_blind_spot.bsd_node import BSDAlertNode  # noqa: E402


class _FakeNode(BSDAlertNode):
    """绕开 rclpy 初始化，手工补齐被测成员，捕获发布负载。"""

    def __init__(self, **params):
        # 参数默认值（可被 params 覆盖）
        cfg = {"warn_enter": 12.0, "warn_exit": 15.0, "approach_enter": 25.0,
               "close_min": 0.5, "valid_max": 40.0, "period": 0.05}
        cfg.update(params)
        for k, v in cfg.items():
            setattr(self, k, v)
        from carla_bsd_blind_spot.bsd_core import BSDController, BSDParams
        self.ctrl = BSDController(BSDParams(**{
            k: cfg[k] for k in ("warn_enter", "warn_exit",
                                "approach_enter", "close_min", "valid_max")}))
        self.valid_max = cfg["valid_max"]
        self._d = {"left": self.valid_max, "right": self.valid_max}
        self._levels = {"left": "normal", "right": "normal"}
        self.alerts = []
        self.events = []

        class _Pub:
            def __init__(self, sink):
                self._sink = sink

            def publish(self, msg):
                self._sink.append(msg.data)

        self.pub_alert = _Pub(self.alerts)
        self.pub_event = _Pub(self.events)

        class _Log:
            def info(self, *a, **k):
                pass

        self.get_logger = lambda: _Log()

    # 测试驱动接口
    def feed(self, left, right):
        self._d["left"] = float(left)
        self._d["right"] = float(right)
        self._step()


class TestBSDAlertNode(unittest.TestCase):

    def test_all_normal_far(self):
        n = _FakeNode()
        n.feed(40.0, 40.0)
        self.assertEqual(json.loads(n.alerts[-1]), {"left": "normal", "right": "normal"})
        self.assertEqual(n.events, [])

    def test_left_warn_on_close(self):
        n = _FakeNode()
        n.feed(10.0, 40.0)
        levels = json.loads(n.alerts[-1])
        self.assertEqual(levels["left"], "warn")
        self.assertEqual(levels["right"], "normal")
        self.assertEqual(n.events, ["left_warn"])

    def test_hysteresis_no_flap(self):
        n = _FakeNode()
        n.feed(10.0, 40.0)          # warn
        n.feed(13.0, 40.0)          # 滞回带内 → 仍 warn，无新事件
        n.feed(14.0, 40.0)
        self.assertEqual(json.loads(n.alerts[-1])["left"], "warn")
        self.assertEqual(n.events, ["left_warn"])          # 事件只发一次
        n.feed(16.0, 40.0)          # >warn_exit → 清除
        self.assertEqual(json.loads(n.alerts[-1])["left"], "normal")
        self.assertEqual(n.events, ["left_warn", "left_clear"])

    def test_right_side_independent(self):
        n = _FakeNode()
        n.feed(40.0, 9.0)
        levels = json.loads(n.alerts[-1])
        self.assertEqual(levels["right"], "warn")
        self.assertEqual(levels["left"], "normal")
        self.assertEqual(n.events, ["right_warn"])

    def test_both_sides_warn(self):
        n = _FakeNode()
        n.feed(8.0, 9.0)
        levels = json.loads(n.alerts[-1])
        self.assertEqual(levels, {"left": "warn", "right": "warn"})
        # 同拍多事件合并为一条逗号分隔消息
        self.assertEqual(
            sorted(e for msg in n.events for e in msg.split(",")),
            ["left_warn", "right_warn"])

    def test_levels_sequence_stable_between_events(self):
        n = _FakeNode()
        n.feed(40.0, 40.0)
        n.feed(39.0, 40.0)
        n.feed(38.5, 40.0)
        # 无跃迁时不产生事件，alert 仍每拍发布
        self.assertEqual(n.events, [])
        self.assertEqual(len(n.alerts), 3)


if __name__ == "__main__":
    unittest.main()
