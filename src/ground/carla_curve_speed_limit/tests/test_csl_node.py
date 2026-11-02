"""CSL 控制节点逻辑单元测试（不依赖 ROS 与 CARLA）。

用桩替代 rclpy/std_msgs，直接驱动 _step 的控制-发布语义：
P 控制律（油门/制动互斥）、斜坡收敛、状态发布、事件去重。
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
                raise RuntimeError("ROS 未安装：本测试只验证控制逻辑，不应触发生命周期")

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
    if not hasattr(msg_mod, "Float32MultiArray"):
        class Float32MultiArray:
            def __init__(self, data=None):
                self.data = data or []
        msg_mod.Float32MultiArray = Float32MultiArray
    if not hasattr(msg_mod, "String"):
        class String:
            def __init__(self, data=""):
                self.data = data
        msg_mod.String = String
    sys.modules["std_msgs.msg"] = msg_mod
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs")).msg = msg_mod


_ensure_ros_stubs()

from carla_curve_speed_limit.csl_node import CSLControlNode  # noqa: E402
from carla_curve_speed_limit.curve_speed_core import (  # noqa: E402
    CSLController, CSLParams)


class _FakeNode(CSLControlNode):
    """绕开 rclpy 初始化，手工补齐被测成员，捕获发布负载。"""

    def __init__(self, **overrides):
        cfg = {"cruise_speed": 8.0, "a_lat_max": 2.5, "look_ahead": 30.0,
               "v_min": 1.5, "decel_max": 2.0, "accel_max": 1.5,
               "slow_enter": 0.5, "slow_exit": 0.2, "valid_max": 30.0,
               "period": 0.05, "kp_throttle": 0.45, "kp_brake": 0.28}
        cfg.update(overrides)
        self.p = CSLParams(**{k: cfg[k] for k in (
            "cruise_speed", "a_lat_max", "look_ahead", "v_min", "decel_max",
            "accel_max", "slow_enter", "slow_exit", "valid_max")})
        self.period = cfg["period"]
        self.kp_throttle = cfg["kp_throttle"]
        self.kp_brake = cfg["kp_brake"]
        self.ctrl = CSLController(self.p)
        self._target = self.p.cruise_speed
        self._speed = 0.0
        self._last_state = self.ctrl._state
        self.controls = []   # [throttle, brake]
        self.states = []
        self.events = []

        class _Pub:
            def __init__(self, sink):
                self._sink = sink

            def publish(self, msg):
                self._sink.append(msg.data)

        self.pub_control = _Pub(self.controls)
        self.pub_state = _Pub(self.states)
        self.pub_event = _Pub(self.events)

        class _Log:
            def info(self, *a, **k):
                pass

        self.get_logger = lambda: _Log()

    def feed(self, target, speed):
        self._target = float(target)
        self._speed = float(speed)
        self._step()


class TestControlLaw(unittest.TestCase):

    def test_brake_when_target_below_speed(self):
        n = _FakeNode()
        n.feed(5.0, 8.0)   # 指令速度约 7.9 < 实测 8.0 → 制动
        th, br = n.controls[-1]
        self.assertEqual(th, 0.0)
        self.assertGreater(br, 0.0)

    def test_throttle_when_target_above_speed(self):
        n = _FakeNode()
        n._target = 8.0
        n._speed = 3.0
        n._step()
        th, br = n.controls[-1]
        self.assertEqual(br, 0.0)
        self.assertGreater(th, 0.0)
        self.assertLessEqual(th, 1.0)

    def test_throttle_never_exceeds_one(self):
        n = _FakeNode()
        n._target = 8.0
        n._speed = 0.0     # 误差 8×0.45=3.6 → 必须截断到 1
        for _ in range(3):
            n._step()
        th, br = n.controls[-1]
        self.assertEqual(th, 1.0)

    def test_brake_scales_with_error(self):
        n = _FakeNode()
        n.feed(5.0, 8.0)
        br_small = n.controls[-1][1]
        n.feed(1.5, 8.0)   # 更大误差（指令趋近 1.5）
        n.feed(1.5, 8.0)
        br_big = n.controls[-1][1]
        self.assertGreater(br_big, br_small)

    def test_throttle_and_brake_mutually_exclusive(self):
        n = _FakeNode()
        for t, s in [(8.0, 2.0), (3.0, 8.0), (8.0, 8.0), (5.0, 6.0)] * 3:
            n.feed(t, s)
            th, br = n.controls[-1]
            self.assertTrue(th == 0.0 or br == 0.0)


class TestStatePublishing(unittest.TestCase):

    def test_state_published_every_tick(self):
        n = _FakeNode()
        n.feed(8.0, 5.0)
        n.feed(8.0, 5.0)
        self.assertEqual(len(n.states), 2)
        self.assertEqual(n.states[0], "cruise")

    def test_transition_event_emitted_once(self):
        n = _FakeNode()
        n.feed(5.0, 20.0)    # cruise → slowdown
        n.feed(5.0, 20.0)    # 保持，无新事件
        self.assertEqual(n.events, ["slowdown_warn"])
        self.assertEqual(n.states[-1], "slowdown")

    def test_recovery_event(self):
        n = _FakeNode()
        n.feed(5.0, 20.0)    # 进入 slowdown
        n._speed = 5.0
        n._target = 7.95     # 差 0.05 < slow_exit → 恢复
        n._step()
        self.assertEqual(n.events[-1], "cruise_resume")
        self.assertEqual(n.states[-1], "cruise")


if __name__ == "__main__":
    unittest.main()
