#!/usr/bin/env python3
"""ROS 话题闭环冒烟测试：回放节点 → MotorImbalanceNode → 输出话题全链路。

覆盖审阅关心的四条基本路径：
  1) 正常输入     —— /drone/motor_commands + /drone/esc_rpm（10 Hz）→
                     /drone/motor_health、/drone/motor_status、
                     /drone/commands_compensated 三路输出，验证基线学习
                     （INIT → MONITORING）与健康输出；
  2) 异常输入     —— 电机 2 转速效率衰减 15%/28% → rpm_tracking_warn /
                     rpm_tracking_emergency → IMBALANCE → EMERGENCY，
                     /drone/emergency_land 置 True；桨损场景（转速正常、
                     0 号指令持续偏高 24%）→ cmd_deviation_warn → IMBALANCE；
  3) 超时行为     —— ESC 遥测与指令话题同时中断 1.5 s → 10 Hz 看门狗
                     stream_timeout 逐拍降级至 IMBALANCE（设计上封顶，
                     不因断流触发紧急降落）；
  4) 恢复行为     —— 电机恢复后连续正常帧 → 恢复 MONITORING，
                     emergency_land 回落 False。

双模式运行（同一份测试代码与同一个 MotorImbalanceNode）：
  * 部署机（Ubuntu + ROS 2）：检测到 rclpy 可导入时，用真实 rclpy 节点、
    真实话题与真实定时器在进程内闭环跑同一场景，墙钟驱动：
        source /opt/ros/humble/setup.bash
        python -m unittest -v test_ros_loop_smoke.py
  * 无 ROS 2 的机器（Windows / 纯 Python CI）：自动回退到本文件内置的
    最小 rclpy 兼容层（进程内话题投递 + 可推进的虚拟时钟，纯标准库），
    场景与断言完全一致且逐拍确定：
        python -m unittest -v test_ros_loop_smoke.py

兼容层只模拟 rclpy 的节点/发布/订阅/定时器/参数/时钟这几个被
MotorImbalanceNode 用到的原语，判据与节点代码原封不动。

场景时间线（与 main.py 离线回放同一套剖面）：
  0–6 s    健康悬停（基线学习 25 帧后冻结）           → INIT → MONITORING
  6–12 s   电机 2 效率 1.00→0.85（跟踪误差 15%）      → IMBALANCE（~7.8 s）
  12–20 s  效率进一步到 0.72（跟踪误差 28%）          → EMERGENCY（~13 s）
  18.4–19.9 s 遥测中断（两路话题都停发）              → stream_timeout
  20–26 s  恢复健康                                    → 恢复 MONITORING
  26–34 s  桨损：转速正常、0 号指令 +24% 悬停指令      → IMBALANCE（~26.4 s）
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

# ---------------------------------------------------------------------------
# 传输层选择：优先真实 rclpy，缺失时安装内置兼容层（shim）
# ---------------------------------------------------------------------------

try:
    import rclpy  # noqa: F401
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter as _RealParameter

    USING_REAL_ROS = True
except ImportError:
    USING_REAL_ROS = False

    _BROKER: dict = {}          # topic -> [(callback, owner_node)]
    _LIVE_NODES: list = []
    _SIM = {"t": 0.0}           # 虚拟时钟（秒）

    class _Time:
        def __init__(self, t: float) -> None:
            self.nanoseconds = int(t * 1e9)

    class _Clock:
        def now(self) -> "_Time":
            return _Time(_SIM["t"])

    class _Logger:
        def __init__(self) -> None:
            self.messages: list = []

        def info(self, msg) -> None:
            self.messages.append(("info", str(msg)))

        def warn(self, msg) -> None:
            self.messages.append(("warn", str(msg)))

    class Bool:
        def __init__(self, data: bool = False) -> None:
            self.data = bool(data)

    class Float32MultiArray:
        def __init__(self, data=None) -> None:
            self.data = list(data) if data is not None else []

    class String:
        def __init__(self, data: str = "") -> None:
            self.data = str(data)

    class _Publisher:
        def __init__(self, topic: str) -> None:
            self._topic = topic

        def publish(self, msg) -> None:
            for cb, _owner in list(_BROKER.get(self._topic, [])):
                cb(msg)

    class _ParamValue:
        def __init__(self, value) -> None:
            self.value = value

    class Parameter:
        def __init__(self, name: str, value=None) -> None:
            self.name = name
            self.value = value

    class Node:
        """最小 rclpy 节点：同步话题投递 + 由 sim_advance 驱动的定时器。"""

        def __init__(self, name: str, *, parameter_overrides=None) -> None:
            self._node_name = name
            self._params = {p.name: p.value for p in (parameter_overrides or [])}
            self._timers: list = []
            self._sub_topics: list = []
            self._logger = _Logger()
            _LIVE_NODES.append(self)

        def declare_parameter(self, name: str, value) -> None:
            self._params.setdefault(name, value)

        def get_parameter(self, name: str) -> _ParamValue:
            return _ParamValue(self._params[name])

        def create_publisher(self, msg_type, topic: str, qos) -> _Publisher:
            return _Publisher(topic)

        def create_subscription(self, msg_type, topic: str, callback, qos) -> None:
            _BROKER.setdefault(topic, []).append((callback, self))
            self._sub_topics.append(topic)

        def create_timer(self, period: float, callback) -> None:
            self._timers.append(
                {"period": period, "cb": callback, "next": _SIM["t"] + period}
            )

        def get_logger(self) -> _Logger:
            return self._logger

        def get_clock(self) -> _Clock:
            return _Clock()

        def destroy_node(self) -> None:
            for topic in self._sub_topics:
                kept = [e for e in _BROKER.get(topic, []) if e[1] is not self]
                if kept:
                    _BROKER[topic] = kept
                else:
                    _BROKER.pop(topic, None)
            if self in _LIVE_NODES:
                _LIVE_NODES.remove(self)

    _rclpy_mod = types.ModuleType("rclpy")
    _rclpy_mod.init = lambda args=None: None
    _rclpy_mod.shutdown = lambda: None
    _rclpy_mod.spin = lambda node: None
    _node_mod = types.ModuleType("rclpy.node")
    _node_mod.Node = Node
    _rclpy_mod.node = _node_mod
    _std_msgs_mod = types.ModuleType("std_msgs")
    _std_msg_mod = types.ModuleType("std_msgs.msg")
    _std_msg_mod.Bool = Bool
    _std_msg_mod.Float32MultiArray = Float32MultiArray
    _std_msg_mod.String = String
    _std_msgs_mod.msg = _std_msg_mod
    sys.modules["rclpy"] = _rclpy_mod
    sys.modules["rclpy.node"] = _node_mod
    sys.modules["std_msgs"] = _std_msgs_mod
    sys.modules["std_msgs.msg"] = _std_msg_mod

    def _sim_advance(to_t: float) -> None:
        """把虚拟时钟推进到 to_t，途中按各自周期触发所有定时器（看门狗）。"""

        while True:
            due = [
                tm
                for nd in _LIVE_NODES
                for tm in nd._timers
                if tm["next"] <= to_t + 1e-9
            ]
            if not due:
                break
            first = min(tm["next"] for tm in due)
            _SIM["t"] = max(first, _SIM["t"])
            for tm in due:
                if tm["next"] <= _SIM["t"] + 1e-9:
                    tm["cb"]()
                    tm["next"] += tm["period"]
        _SIM["t"] = to_t

    _rclpy_mod.sim_advance = _sim_advance

if USING_REAL_ROS:
    from rclpy.node import Node as _RclpyNode  # noqa: E402
    from std_msgs.msg import Bool, Float32MultiArray, String  # noqa: E402

# 传输层就绪后再导入节点模块（节点判据代码不变，只走话题链路）
from uav_motor_thrust_imbalance_monitor.motor_imbalance_node import (  # noqa: E402
    MotorImbalanceNode,
)

# ---------------------------------------------------------------------------
# 场景数据（与 main.py 离线回放同一套参数与故障剖面）
# ---------------------------------------------------------------------------

HOVER = 0.5
RPM_MAX = 8000.0
N = 4
DT = 0.1
FLIGHT_END = 34.0
DROPOUT = (18.4, 19.9)          # 两路话题都停发的窗口


def motor_efficiency(t: float) -> float:
    """电机 2 转速效率（与 main.py 一致）。"""

    if t < 6.0:
        return 1.0
    if t < 8.0:
        return 1.0 - 0.15 * (t - 6.0) / 2.0
    if t < 12.0:
        return 0.85
    if t < 13.0:
        return 0.85 - 0.13 * (t - 12.0)
    if t < 20.0:
        return 0.72
    return 1.0


def mixer_commands(t: float):
    """与 main.py 相同的控制器/桨损指令剖面。"""

    cmds = [HOVER] * N
    eff = motor_efficiency(t)
    if eff < 1.0:
        need = HOVER / eff
        cmds[2] = min(1.0, HOVER + 0.5 * (need - HOVER))
    if t >= 26.0:
        cmds[0] = HOVER + 0.24 * HOVER
    return cmds


def esc_rpm(t: float, cmds):
    eff = motor_efficiency(t)
    return [
        cmds[i] * RPM_MAX * (eff if i == 2 else 1.0)
        * (1.0 + 0.005 * math.sin(37.0 * t + i))
        for i in range(N)
    ]


# ---------------------------------------------------------------------------
# 两种传输层的测试挂具
# ---------------------------------------------------------------------------

class _ShimHarness:
    """兼容层模式：虚拟时钟驱动，同步投递，逐拍确定。"""

    def __init__(self, overrides: dict) -> None:
        _SIM["t"] = 0.0
        self.node = MotorImbalanceNode(
            parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()]
        )
        self.hn = Node("smoke_harness")
        self._pub_cmds = self.hn.create_publisher(
            Float32MultiArray, "/drone/motor_commands", 10
        )
        self._pub_rpm = self.hn.create_publisher(
            Float32MultiArray, "/drone/esc_rpm", 10
        )
        self.records: list = []   # (t, health, emergency, status_dict, comp)
        self._pending: dict = {}
        self.hn.create_subscription(
            Bool, "/drone/motor_health", self._on_health, 10)
        self.hn.create_subscription(
            Bool, "/drone/emergency_land", self._on_emg, 10)
        self.hn.create_subscription(
            Float32MultiArray, "/drone/commands_compensated",
            self._on_comp, 10)
        self.hn.create_subscription(
            String, "/drone/motor_status", self._on_status, 10)

    def _on_health(self, msg) -> None:
        self._pending["health"] = msg.data

    def _on_emg(self, msg) -> None:
        self._pending["emergency"] = msg.data

    def _on_comp(self, msg) -> None:
        self._pending["comp"] = list(msg.data)

    def _on_status(self, msg) -> None:
        self.records.append(
            (_SIM["t"], self._pending.get("health"),
             self._pending.get("emergency"), json.loads(msg.data),
             self._pending.get("comp"))
        )
        self._pending = {}

    def publish_frame(self, t: float, cmds, rpm) -> None:
        _sim_advance(t)
        if rpm is not None:
            self._pub_rpm.publish(Float32MultiArray(rpm))
        self._pub_cmds.publish(Float32MultiArray(cmds))

    def advance(self, to_t: float) -> None:
        """只推进时间不发帧（模拟遥测中断，看门狗应触发）。"""

        _sim_advance(to_t)

    def close(self) -> None:
        self.node.destroy_node()
        self.hn.destroy_node()


class _RealHarness:
    """真实 rclpy 模式：后台单线程执行器，墙钟驱动同一场景。"""

    def __init__(self, overrides: dict) -> None:
        rclpy.init()
        self.node = MotorImbalanceNode(
            parameter_overrides=[
                _RealParameter(k, value=v) for k, v in overrides.items()
            ]
        )
        self.hn = _RclpyNode("smoke_harness")
        self.records: list = []
        self._pending: dict = {}
        self._t0 = time.monotonic()
        self.hn.create_subscription(
            Bool, "/drone/motor_health", self._on_health, 10)
        self.hn.create_subscription(
            Bool, "/drone/emergency_land", self._on_emg, 10)
        self.hn.create_subscription(
            Float32MultiArray, "/drone/commands_compensated",
            self._on_comp, 10)
        self.hn.create_subscription(
            String, "/drone/motor_status", self._on_status, 10)
        self._pub_cmds = self.hn.create_publisher(
            Float32MultiArray, "/drone/motor_commands", 10
        )
        self._pub_rpm = self.hn.create_publisher(
            Float32MultiArray, "/drone/esc_rpm", 10
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._executor.add_node(self.hn)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _on_health(self, msg) -> None:
        self._pending["health"] = msg.data

    def _on_emg(self, msg) -> None:
        self._pending["emergency"] = msg.data

    def _on_comp(self, msg) -> None:
        self._pending["comp"] = list(msg.data)

    def _on_status(self, msg) -> None:
        self.records.append(
            (time.monotonic() - self._t0, self._pending.get("health"),
             self._pending.get("emergency"), json.loads(msg.data),
             self._pending.get("comp"))
        )
        self._pending = {}

    def publish_frame(self, t: float, cmds, rpm) -> None:
        # 参数 t 只表达场景节奏，节点时间戳取真实时钟
        if rpm is not None:
            self._pub_rpm.publish(Float32MultiArray(rpm))
            time.sleep(DT / 10.0)
        self._pub_cmds.publish(Float32MultiArray(cmds))
        time.sleep(DT - DT / 10.0)

    def advance(self, to_t: float) -> None:
        """遥测中断：只等待一拍，看门狗靠真实定时器触发。"""

        time.sleep(DT)

    def close(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=5.0)
        self.node.destroy_node()
        self.hn.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# 测试用例
# ---------------------------------------------------------------------------

class RosLoopSmokeTest(unittest.TestCase):
    """话题闭环冒烟测试。真实 rclpy 不可用时自动使用内置兼容层。"""

    def setUp(self) -> None:
        harness_cls = _RealHarness if USING_REAL_ROS else _ShimHarness
        self.h = harness_cls({})
        self.mode = "rclpy" if USING_REAL_ROS else "shim"

    def tearDown(self) -> None:
        self.h.close()

    # --- 场景驱动 -------------------------------------------------------

    def _run_full_flight(self):
        """全场景：健康 → 电机衰减 → 深度衰减/紧急 → 中断 → 恢复 → 桨损。"""

        t = 0.0
        while t <= FLIGHT_END + 1e-9:
            cmds = mixer_commands(t)
            rpm = esc_rpm(t, cmds)
            if DROPOUT[0] <= t < DROPOUT[1]:
                self.h.advance(t)         # 两路话题同时中断
            else:
                self.h.publish_frame(t, cmds, rpm)
            t = round(t + DT, 4)
        return self.h.records

    def _run_rpm_stream_loss_only(self):
        """指令话题正常、ESC 转速停发：判据应在无转速分支继续工作。"""

        t = 0.0
        while t < 8.0 + 1e-9:
            cmds = mixer_commands(t)
            rpm = esc_rpm(t, cmds)
            if t < 4.0:
                self.h.publish_frame(t, cmds, rpm)
            else:
                self.h.publish_frame(t, cmds, None)
            t = round(t + DT, 4)
        return self.h.records

    def _run_pure_dropout(self):
        """健康悬停后两路话题同时中断 2.5 s（无动力故障掺杂）。"""

        t = 0.0
        while t < 5.0 + 1e-9:
            cmds = mixer_commands(t)
            rpm = esc_rpm(t, cmds)
            self.h.publish_frame(t, cmds, rpm)
            t = round(t + DT, 4)
        while t < 7.5 + 1e-9:
            self.h.advance(t)
            t = round(t + DT, 4)
        return self.h.records

    # --- 断言工具 -------------------------------------------------------

    def _first_with(self, rec, pred, lo=0.0, hi=1e9):
        for t, _h, _e, s, _c in rec:
            if lo <= t <= hi and pred(s):
                return t
        return None

    # --- 用例 -----------------------------------------------------------

    def test_normal_input_learns_baseline_and_stays_healthy(self):
        """正常段：INIT → MONITORING，healthy=True，补偿为 0。"""

        t = 0.0
        while t < 6.0:
            cmds = mixer_commands(t)
            rpm = esc_rpm(t, cmds)
            self.h.publish_frame(t, cmds, rpm)
            t = round(t + DT, 4)
        rec = self.h.records
        self.assertTrue(rec, "闭环没有产生任何输出")
        states = [s["state"] for _t, _h, _e, s, _c in rec]
        self.assertIn("MONITORING", states, f"应完成基线学习：{states[:5]}")
        for _t, health, _e, s, comp in rec[-5:]:
            self.assertTrue(health, "健康段 motor_health 应为 True")
            self.assertEqual(s["reason"], "ok")
            self.assertIsNotNone(comp)
            self.assertEqual(comp, [0.5, 0.5, 0.5, 0.5],
                             "健康段补偿指令应等于原指令")

    def test_rpm_degradation_warns_then_emergency(self):
        """电机衰减：15% 跟踪误差 → IMBALANCE；28% → EMERGENCY + 紧急降落。"""

        rec = self._run_full_flight()
        t_imb = self._first_with(
            rec, lambda s: s["state"] == "IMBALANCE", lo=7.0, hi=10.0
        )
        self.assertIsNotNone(
            t_imb, f"[{self.mode}] 8 s 起的效率衰减应在 ~8 s 触发 IMBALANCE"
        )
        t_emg = self._first_with(
            rec, lambda s: s["state"] == "EMERGENCY", lo=12.0, hi=16.0
        )
        self.assertIsNotNone(
            t_emg, f"[{self.mode}] 13 s 深度衰减应触发 EMERGENCY"
        )
        seen = False
        for _t, health, emergency, s, _c in rec:
            if s["state"] == "EMERGENCY" and 14.0 <= _t <= 18.0:
                self.assertTrue(emergency, "EMERGENCY 应发布紧急降落建议")
                self.assertFalse(health)
                self.assertEqual(s["worst_motor"], 2)
                seen = True
                break
        self.assertTrue(seen, "深度衰减段应有 EMERGENCY 输出")
        # IMBALANCE 段应有补偿：电机 2 指令被抬升
        comp_seen = False
        for _t, _h, _e, s, comp in rec:
            if s["state"] == "IMBALANCE" and comp is not None and comp[2] > 0.5:
                comp_seen = True
                break
        self.assertTrue(comp_seen, "IMBALANCE 段应抬升衰减电机的补偿指令")

    def test_esc_stream_timeout_watchdog_caps_at_imbalance(self):
        """纯断流（无动力故障）：看门狗逐拍降级至 IMBALANCE，不触发紧急降落。"""

        rec = self._run_pure_dropout()
        t_to = self._first_with(
            rec, lambda s: "stream_timeout" in s["reason"], lo=5.8, hi=7.0
        )
        self.assertIsNotNone(
            t_to, f"[{self.mode}] 停发 1 s 后应出现 stream_timeout"
        )
        t_imb = self._first_with(
            rec, lambda s: s["state"] == "IMBALANCE", lo=5.8, hi=7.5
        )
        self.assertIsNotNone(
            t_imb, f"[{self.mode}] 持续断流应连帧降级到 IMBALANCE"
        )
        for _t, health, emergency, s, _c in rec:
            if "stream_timeout" in s["reason"]:
                self.assertFalse(emergency,
                                 "纯断流不得触发紧急降落（安全设计）")
                self.assertFalse(health)

    def test_full_flight_dropout_leaves_watchdog_trace(self):
        """全场景中的中断窗口应同样出现 stream_timeout（与深度衰减叠加）。"""

        rec = self._run_full_flight()
        lo, hi = DROPOUT[0] + 0.8, DROPOUT[1] + 0.4
        t_to = self._first_with(
            rec, lambda s: "stream_timeout" in s["reason"], lo=lo, hi=hi
        )
        self.assertIsNotNone(
            t_to, f"[{self.mode}] 断流后应在 [{lo:.1f},{hi:.1f}] s 出现 stream_timeout"
        )

    def test_prop_loss_detected_by_command_deviation(self):
        """桨损：转速正常但 0 号指令持续偏高 → cmd_deviation → IMBALANCE。"""

        rec = self._run_full_flight()
        t_prop = self._first_with(
            rec,
            lambda s: "cmd_deviation" in s["reason"],
            lo=26.0, hi=29.0,
        )
        self.assertIsNotNone(
            t_prop, f"[{self.mode}] 26 s 起的桨损应触发指令偏移判据"
        )
        for _t, _h, _e, s, _c in rec:
            if "cmd_deviation" in s["reason"]:
                self.assertEqual(s["worst_motor"], 0)
                self.assertLess(s["track_err_pct"][0], 5.0,
                                "桨损段转速跟踪应基本正常")
                break

    def test_recovery_after_degradation(self):
        """电机恢复后应回到 MONITORING，紧急降落建议回落。"""

        rec = self._run_full_flight()
        t_rec = self._first_with(
            rec, lambda s: s["reason"] == "recovered", lo=19.5, hi=23.0
        )
        self.assertIsNotNone(
            t_rec, f"[{self.mode}] 20 s 电机恢复后应回到 MONITORING"
        )
        for _t, health, emergency, s, _c in rec:
            if s["reason"] == "recovered":
                self.assertTrue(health)
                self.assertFalse(emergency)
                break

    def test_missing_rpm_telemetry_keeps_monitoring(self):
        """只有指令、无转速遥测：基线可学习、健康保持（降级观测但不误报）。"""

        rec = self._run_rpm_stream_loss_only()
        self.assertTrue(rec)
        tail = [entry for entry in rec if entry[0] >= 6.0]
        self.assertTrue(tail, "应有 6 s 之后的输出")
        for _t, health, _e, s, _c in tail:
            self.assertEqual(s["state"], "MONITORING")
            self.assertFalse(s["rpm_available"])
            self.assertTrue(health)


if __name__ == "__main__":
    unittest.main(verbosity=2)
