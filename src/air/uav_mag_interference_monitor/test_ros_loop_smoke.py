#!/usr/bin/env python3
"""ROS 话题闭环冒烟测试：回放节点 → MagInterferenceNode → 输出话题全链路。

审阅关注的四条基本路径全部覆盖：
  1) 正常输入     —— /drone/magnetic_field（sensor_msgs/MagneticField，特斯拉）
                     + /drone/gnss（sensor_msgs/NavSatFix）→ /drone/heading 等
                     三路输出，验证单位换算（T→µT）、航迹向解算与 TRUSTED 输出；
  2) 异常输入     —— 横向磁场干扰推偏磁航向 → DEGRADED → UNRELIABLE 并切换
                     GNSS 航迹向；沿航向干扰抬高模值 → field_norm_out_of_range；
                     NaN 非有限值输入 → non_finite 降级；
  3) 超时行为     —— 磁场话题中断（GNSS 正常）→ 10 Hz 看门狗按 stale_timeout
                     逐拍降级至 UNRELIABLE，期间输出 GNSS 航迹向；悬停（地速
                     低于门限、航迹向不可用）时冻结最后可信航向并置 valid=False；
  4) 恢复行为     —— 干扰退去后连续 recovery_streak 帧正常 → 回到 TRUSTED。

双模式运行（同一份测试代码与同一个 MagInterferenceNode）：
  * 部署机（Ubuntu + ROS 2）：检测到 rclpy 可导入时，用真实 rclpy 节点、
    真实话题与真实定时器在进程内闭环跑同一场景，墙钟驱动，全程约 50 s：
        source /opt/ros/humble/setup.bash
        python -m unittest -v test_ros_loop_smoke.py
  * 无 ROS 2 的机器（Windows / 纯 Python CI）：自动回退到本文件内置的
    最小 rclpy 兼容层（进程内话题投递 + 可推进的虚拟时钟，纯标准库），
    场景与断言完全一致且逐拍确定：
        python -m unittest -v test_ros_loop_smoke.py

兼容层只模拟 rclpy 的节点/发布/订阅/定时器/参数/时钟这几个被
MagInterferenceNode 用到的原语，判据与节点代码原封不动。

场景时间线（与 main.py 离线回放同一套干扰剖面）：
  0–8 s    正常磁场，东北向 4.5 m/s 平飞                → TRUSTED / mag
  8–20 s   横向干扰 0→13 µT 推偏磁航向（峰值约 30°）    → DEGRADED → UNRELIABLE / gnss_course
  22.3–24.8 s 磁场话题中断 2.5 s（GNSS 正常 10 Hz）     → stream_timeout → UNRELIABLE / gnss_course
  26–32 s  沿航向干扰 18 µT 抬高模值（约 +80%）         → field_norm_out_of_range → UNRELIABLE
  32 s 之后 干扰退去                                     → 5 帧内恢复 TRUSTED / mag
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

    class _Stamp:
        def __init__(self, t: float = 0.0) -> None:
            self.sec = int(t)
            self.nanosec = int(round((t - int(t)) * 1e9))

    class _Header:
        def __init__(self) -> None:
            self.stamp = _Stamp(0.0)
            self.frame_id = ""

    class _Vector3:
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class MagneticField:
        def __init__(self) -> None:
            self.header = _Header()
            self.magnetic_field = _Vector3()

    class NavSatFix:
        def __init__(self) -> None:
            self.header = _Header()
            self.latitude = 0.0
            self.longitude = 0.0
            self.altitude = 0.0

    class Bool:
        def __init__(self, data: bool = False) -> None:
            self.data = bool(data)

    class Float64:
        def __init__(self, data: float = 0.0) -> None:
            self.data = float(data)

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
    _sensor_msgs_mod = types.ModuleType("sensor_msgs")
    _sensor_msg_mod = types.ModuleType("sensor_msgs.msg")
    _sensor_msg_mod.MagneticField = MagneticField
    _sensor_msg_mod.NavSatFix = NavSatFix
    _sensor_msgs_mod.msg = _sensor_msg_mod
    _std_msgs_mod = types.ModuleType("std_msgs")
    _std_msg_mod = types.ModuleType("std_msgs.msg")
    _std_msg_mod.Bool = Bool
    _std_msg_mod.Float64 = Float64
    _std_msg_mod.String = String
    _std_msgs_mod.msg = _std_msg_mod
    sys.modules["rclpy"] = _rclpy_mod
    sys.modules["rclpy.node"] = _node_mod
    sys.modules["sensor_msgs"] = _sensor_msgs_mod
    sys.modules["sensor_msgs.msg"] = _sensor_msg_mod
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
    from sensor_msgs.msg import MagneticField, NavSatFix  # noqa: E402
    from std_msgs.msg import Bool, Float64, String  # noqa: E402

# 传输层就绪后再导入节点模块（节点判据代码不变，只走话题链路）
from uav_mag_interference_monitor.mag_interference_node import (  # noqa: E402
    MagInterferenceNode,
)
from uav_mag_interference_monitor import fit_circle  # noqa: E402
from main import build_preflight_samples, interference_profile  # noqa: E402

# ---------------------------------------------------------------------------
# 场景数据（与 main.py 离线回放同一套参数与干扰剖面）
# ---------------------------------------------------------------------------

COURSE = math.radians(45.0)     # 真实航向 / 航迹向：东北 45°
SPEED = 4.5                     # 地速 m/s（> min_ground_speed=2）
DT = 0.1                        # 话题周期 10 Hz
FLIGHT_END = 34.0
DROPOUT = (22.3, 24.8)          # 磁场话题中断区间（GNSS 照常发布）
LAT0, LON0 = 30.0, 114.0
M_PER_DEG_LAT = 110540.0
M_PER_DEG_LON = 111320.0


def _calibration():
    """用与离线回放相同的怠速整圈样本做圆拟合，得到偏差与半径基准。"""

    idle = build_preflight_samples(60, (0.0, 0.0, 0.0), seed=0)
    return fit_circle(idle)


def _field(t: float, bias_xy):
    """t 时刻机体坐标系下的磁场（µT），叠加与离线回放一致的干扰剖面。"""

    cross, along = interference_profile(t)
    hx, hy = math.cos(COURSE), math.sin(COURSE)   # 沿航向单位矢量
    cx, cy = -hy, hx                              # 横向（左法向）
    mx = 22.0 * hx + bias_xy[0] + cross * cx + along * hx
    my = 22.0 * hy + bias_xy[1] + cross * cy + along * hy
    mz = -42.0 + 1.2
    return mx, my, mz


def _latlon(t: float):
    """按恒定航迹向与地速积分出的 GNSS 位置。"""

    north = SPEED * math.cos(COURSE) * t
    east = SPEED * math.sin(COURSE) * t
    return LAT0 + north / M_PER_DEG_LAT, LON0 + east / (
        M_PER_DEG_LON * math.cos(math.radians(LAT0))
    )


def _raw_mag_heading_deg(mx: float, my: float, bias_xy) -> float:
    """未门控的原始磁航向（度），用于对照。"""

    return math.degrees(math.atan2(my - bias_xy[1], mx - bias_xy[0]))


# ---------------------------------------------------------------------------
# 两种传输层的测试挂具
# ---------------------------------------------------------------------------

class _ShimHarness:
    """兼容层模式：虚拟时钟驱动，同步投递，逐拍确定。"""

    def __init__(self, overrides: dict) -> None:
        _SIM["t"] = 0.0
        self.node = MagInterferenceNode(
            parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()]
        )
        self.hn = Node("smoke_harness")
        self._pub_mag = self.hn.create_publisher(
            MagneticField, "/drone/magnetic_field", 10
        )
        self._pub_gnss = self.hn.create_publisher(NavSatFix, "/drone/gnss", 10)
        self.records: list = []   # (t, heading_rad, valid, status_dict)
        self._pending: dict = {}
        self.hn.create_subscription(Float64, "/drone/heading", self._on_heading, 10)
        self.hn.create_subscription(Bool, "/drone/heading_valid", self._on_valid, 10)
        self.hn.create_subscription(String, "/drone/mag_status", self._on_status, 10)

    def _on_heading(self, msg) -> None:
        self._pending["heading"] = msg.data

    def _on_valid(self, msg) -> None:
        self._pending["valid"] = msg.data

    def _on_status(self, msg) -> None:
        self.records.append(
            (_SIM["t"], self._pending.get("heading"), self._pending.get("valid"),
             json.loads(msg.data))
        )
        self._pending = {}

    def publish_mag(self, t: float, mx: float, my: float, mz: float) -> None:
        _sim_advance(t)
        msg = MagneticField()
        msg.header.stamp = _Stamp(t)
        msg.magnetic_field.x = mx * 1e-6   # 判据层用 µT，话题用特斯拉
        msg.magnetic_field.y = my * 1e-6
        msg.magnetic_field.z = mz * 1e-6
        self._pub_mag.publish(msg)

    def publish_gnss(self, t: float, lat: float, lon: float) -> None:
        _sim_advance(t)
        msg = NavSatFix()
        msg.header.stamp = _Stamp(t)
        msg.latitude = lat
        msg.longitude = lon
        self._pub_gnss.publish(msg)

    def advance(self, to_t: float) -> None:
        """只推进时间不发磁场（模拟话题中断，看门狗应触发）。"""

        _sim_advance(to_t)

    def close(self) -> None:
        self.node.destroy_node()
        self.hn.destroy_node()


class _RealHarness:
    """真实 rclpy 模式：后台单线程执行器，墙钟驱动同一场景。"""

    def __init__(self, overrides: dict) -> None:
        rclpy.init()
        self.node = MagInterferenceNode(
            parameter_overrides=[
                _RealParameter(k, value=v) for k, v in overrides.items()
            ]
        )
        self.hn = _RclpyNode("smoke_harness")
        self.records: list = []
        self._pending: dict = {}
        self._t0 = time.monotonic()
        self.hn.create_subscription(Float64, "/drone/heading", self._on_heading, 10)
        self.hn.create_subscription(Bool, "/drone/heading_valid", self._on_valid, 10)
        self.hn.create_subscription(String, "/drone/mag_status", self._on_status, 10)
        self._pub_mag = self.hn.create_publisher(
            MagneticField, "/drone/magnetic_field", 10
        )
        self._pub_gnss = self.hn.create_publisher(NavSatFix, "/drone/gnss", 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._executor.add_node(self.hn)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _stamp_now(self, msg) -> None:
        now = self.hn.get_clock().now()
        msg.header.stamp.sec = now.nanoseconds // 1_000_000_000
        msg.header.stamp.nanosec = now.nanoseconds % 1_000_000_000

    def _on_heading(self, msg) -> None:
        self._pending["heading"] = msg.data

    def _on_valid(self, msg) -> None:
        self._pending["valid"] = msg.data

    def _on_status(self, msg) -> None:
        self.records.append(
            (time.monotonic() - self._t0, self._pending.get("heading"),
             self._pending.get("valid"), json.loads(msg.data))
        )
        self._pending = {}

    def publish_mag(self, t: float, mx: float, my: float, mz: float) -> None:
        # 参数 t 只表达场景节奏（帧间隔由 advance/sleep 控制），时间戳取真实时钟
        msg = MagneticField()
        self._stamp_now(msg)
        msg.magnetic_field.x = mx * 1e-6
        msg.magnetic_field.y = my * 1e-6
        msg.magnetic_field.z = mz * 1e-6
        self._pub_mag.publish(msg)
        time.sleep(DT)

    def publish_gnss(self, t: float, lat: float, lon: float) -> None:
        msg = NavSatFix()
        self._stamp_now(msg)
        msg.latitude = lat
        msg.longitude = lon
        self._pub_gnss.publish(msg)

    def advance(self, to_t: float) -> None:
        """磁场断流：只等待一拍，看门狗靠真实定时器触发。"""

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
        cal = _calibration()
        self.bias = cal.bias
        self.radius = cal.radius
        self.overrides = {
            "bias_x": cal.bias[0],
            "bias_y": cal.bias[1],
            "expected_radius": cal.radius,
        }
        harness_cls = _RealHarness if USING_REAL_ROS else _ShimHarness
        self.h = harness_cls(self.overrides)
        self.mode = "rclpy" if USING_REAL_ROS else "shim"

    def tearDown(self) -> None:
        self.h.close()

    # --- 场景驱动 -------------------------------------------------------

    def _run_full_flight(self):
        """全场景回放：正常 → 横向干扰 → 磁场断流 → 模值异常 → 恢复。"""

        t = 0.0
        while t <= FLIGHT_END + 1e-9:
            mx, my, mz = _field(t, self.bias)
            lat, lon = _latlon(t)
            self.h.publish_gnss(t, lat, lon)
            if DROPOUT[0] <= t < DROPOUT[1]:
                self.h.advance(t)     # 磁场话题中断：GNSS 照常，只有磁场不发
            else:
                self.h.publish_mag(t, mx, my, mz)
            t = round(t + DT, 4)
        return self.h.records

    def _run_flight_with_nan(self):
        """平飞中磁力计输出 NaN：3 s 正常，随后 3 s 全部非有限值。"""

        t = 0.0
        while t < 6.0:
            lat, lon = _latlon(t)
            self.h.publish_gnss(t, lat, lon)
            if t >= 3.0:
                self.h.publish_mag(t, float("nan"), float("nan"), float("nan"))
            else:
                mx, my, mz = _field(0.0, self.bias)
                self.h.publish_mag(t, mx, my, mz)
            t = round(t + DT, 4)
        return self.h.records

    def _run_hover_dropout(self):
        """先平飞建立可信航向，再悬停并断流磁场（地速 0 → 航迹向不可用）。"""

        t = 0.0
        while t < 3.0:                       # 平飞段：建立 TRUSTED 与航迹向
            mx, my, mz = _field(0.0, self.bias)
            lat, lon = _latlon(t)
            self.h.publish_gnss(t, lat, lon)
            self.h.publish_mag(t, mx, my, mz)
            t = round(t + DT, 4)
        while t < 8.0:                       # 悬停段：GNSS 位置不变 → 地速 0
            mx, my, mz = _field(0.0, self.bias)
            self.h.publish_gnss(t, LAT0, LON0)
            self.h.publish_mag(t, mx, my, mz)
            t = round(t + DT, 4)
        while t < 10.5:                      # 磁场断流 2.5 s，GNSS 仍发布
            self.h.publish_gnss(t, LAT0, LON0)
            self.h.advance(t)
            t = round(t + DT, 4)
        return self.h.records

    # --- 断言工具 -------------------------------------------------------

    def _first_with(self, rec, pred, lo=0.0, hi=1e9):
        for t, _h, _v, s in rec:
            if lo <= t <= hi and pred(s):
                return t
        return None

    # --- 用例 -----------------------------------------------------------

    def test_normal_input_trusted_with_correct_units(self):
        """正常段：TRUSTED、来源 mag、航向≈45°、模值≈100%（证明 T→µT 正确）。"""

        t = 0.0
        while t < 6.0:
            mx, my, mz = _field(0.0, self.bias)
            lat, lon = _latlon(t)
            self.h.publish_gnss(t, lat, lon)
            self.h.publish_mag(t, mx, my, mz)
            t = round(t + DT, 4)
        rec = self.h.records
        self.assertTrue(rec, "闭环没有产生任何输出")
        tail = [entry for entry in rec if entry[0] >= 2.0]
        self.assertTrue(tail, "应有 2 s 之后的输出")
        self.assertTrue(
            all(s["state"] == "TRUSTED" for _t, _h, _v, s in tail),
            f"正常段应全部 TRUSTED：{[s['state'] for _t,_h,_v,s in tail]}",
        )
        for _t, heading, valid, s in rec[-5:]:
            self.assertTrue(valid)
            self.assertEqual(s["source"], "mag")
            err = abs(math.degrees(heading) - 45.0)
            self.assertLess(err, 3.0,
                            f"航向应≈45°，实际 {math.degrees(heading):.1f}°")
            self.assertTrue(
                90.0 <= s["field_norm_pct"] <= 110.0,
                f"模值应在 100% 附近（单位换算应正确），实际 {s['field_norm_pct']}%",
            )

    def test_lateral_interference_switches_to_gnss_course(self):
        """横向干扰：磁航向被推偏 → DEGRADED → UNRELIABLE 并改用航迹向。"""

        rec = self._run_full_flight()
        t_deg = self._first_with(
            rec, lambda s: s["state"] == "DEGRADED", lo=7.5, hi=12.0
        )
        self.assertIsNotNone(
            t_deg, f"[{self.mode}] 8 s 起的横向干扰应在 ~9 s 触发 DEGRADED"
        )
        t_unr = self._first_with(
            rec,
            lambda s: (s["state"] == "UNRELIABLE"
                       and s["source"] == "gnss_course"),
            lo=8.0,
            hi=13.5,
        )
        self.assertIsNotNone(
            t_unr, f"[{self.mode}] 持续干扰应在 ~10 s 升级 UNRELIABLE 并切航迹向"
        )
        checked = False
        for _t, heading, valid, s in rec:
            if s["state"] == "UNRELIABLE" and 10.0 <= _t <= 18.0:
                self.assertTrue(valid, "有航迹向时 UNRELIABLE 输出仍应可用")
                err = abs(math.degrees(heading) - 45.0)
                self.assertLess(
                    err, 5.0,
                    f"切航迹向后航向应≈45°，实际 {math.degrees(heading):.1f}°",
                )
                checked = True
                break
        self.assertTrue(checked, "干扰峰值段应有 UNRELIABLE 输出")
        raw_err = abs(
            _raw_mag_heading_deg(*_field(15.0, self.bias)[:2], self.bias) - 45.0
        )
        self.assertGreater(
            raw_err, 20.0,
            f"t=15 s 横向干扰应把原始磁航向推偏>20°，实际 {raw_err:.1f}°",
        )

    def test_mag_stream_timeout_watchdog(self):
        """磁场断流（GNSS 正常）：看门狗逐拍降级，输出 stream_timeout。"""

        rec = self._run_full_flight()
        lo, hi = DROPOUT[0] + 0.9, DROPOUT[1] + 0.9
        t_to = self._first_with(
            rec, lambda s: "stream_timeout" in s["reason"], lo=lo, hi=hi
        )
        self.assertIsNotNone(
            t_to, f"[{self.mode}] 断流后应在 [{lo:.1f},{hi:.1f}] s 出现 stream_timeout"
        )
        t_unr = self._first_with(
            rec,
            lambda s: (s["state"] == "UNRELIABLE"
                       and s["reason"] == "stream_timeout"),
            lo=lo,
            hi=hi + 1.5,
        )
        self.assertIsNotNone(
            t_unr,
            f"[{self.mode}] 断流 2.5 s 应累计到 unreliable_limit 进入 UNRELIABLE",
        )
        for _t, heading, valid, s in rec:
            if s["reason"] == "stream_timeout" and s["state"] == "UNRELIABLE":
                self.assertEqual(s["source"], "gnss_course")
                self.assertTrue(valid)
                err = abs(math.degrees(heading) - 45.0)
                self.assertLess(err, 5.0)
                break

    def test_field_norm_anomaly_degrades(self):
        """沿航向干扰：模值抬高 → field_norm_out_of_range → UNRELIABLE。"""

        rec = self._run_full_flight()
        t_bad = self._first_with(
            rec,
            lambda s: "field_norm_out_of_range" in s["reason"],
            lo=25.5,
            hi=29.0,
        )
        self.assertIsNotNone(
            t_bad, f"[{self.mode}] 26 s 起的模值干扰应触发 field_norm_out_of_range"
        )
        t_unr = self._first_with(
            rec,
            lambda s: (s["state"] == "UNRELIABLE"
                       and "field_norm_out_of_range" in s["reason"]),
            lo=26.0,
            hi=29.5,
        )
        self.assertIsNotNone(t_unr, f"[{self.mode}] 模值持续异常应升级 UNRELIABLE")
        last = rec[-1][3]
        self.assertEqual(last["state"], "TRUSTED", f"结尾应恢复 TRUSTED：{last}")
        self.assertEqual(last["source"], "mag")

    def test_recovery_after_interference(self):
        """横向干扰退去后 5 帧内恢复 TRUSTED，来源回到 mag。"""

        rec = self._run_full_flight()
        t_rec = self._first_with(
            rec, lambda s: s["reason"] == "recovered", lo=19.0, hi=22.5
        )
        self.assertIsNotNone(
            t_rec, f"[{self.mode}] 20 s 干扰退去后应恢复 TRUSTED"
        )

    def test_non_finite_mag_input_degrades(self):
        """NaN 磁力计输入：non_finite 降级，有航迹向时输出仍可用。"""

        rec = self._run_flight_with_nan()
        t_nan = self._first_with(
            rec, lambda s: s["reason"] == "non_finite", lo=2.5, hi=6.5
        )
        self.assertIsNotNone(
            t_nan, f"[{self.mode}] NaN 输入应以 non_finite 降级"
        )
        t_unr = self._first_with(
            rec,
            lambda s: s["state"] == "UNRELIABLE" and s["reason"] == "non_finite",
            lo=3.0,
            hi=6.5,
        )
        self.assertIsNotNone(
            t_unr, f"[{self.mode}] NaN 持续输入应升级 UNRELIABLE"
        )
        for _t, heading, valid, s in rec:
            if s["reason"] == "non_finite":
                self.assertEqual(s["source"], "gnss_course")
                self.assertTrue(valid, "有航迹向兜底时输出应可用")
                err = abs(math.degrees(heading) - 45.0)
                self.assertLess(err, 5.0)
                break

    def test_timeout_without_course_freezes_heading(self):
        """悬停断流：无航迹向 → 冻结最后可信航向并置 valid=False。"""

        rec = self._run_hover_dropout()
        frozen = None
        for t, heading, valid, s in rec:
            if s["state"] == "TRUSTED" and 7.0 <= t <= 8.05:
                frozen = heading
        self.assertIsNotNone(
            frozen, f"[{self.mode}] 断流前悬停段应处于 TRUSTED"
        )
        t_to = self._first_with(
            rec, lambda s: "stream_timeout" in s["reason"], lo=8.8, hi=11.0
        )
        self.assertIsNotNone(
            t_to, f"[{self.mode}] 悬停断流应触发看门狗 stream_timeout"
        )
        found = False
        for _t, heading, valid, s in rec:
            if s["state"] == "UNRELIABLE" and s["reason"] == "stream_timeout":
                self.assertEqual(s["source"], "last_trusted")
                self.assertFalse(valid, "无航迹向兜底时输出应标记不可用")
                self.assertIsNotNone(heading)
                err = abs(math.degrees(heading) - math.degrees(frozen))
                self.assertLess(err, 1.0, "应冻结最后可信航向")
                found = True
                break
        self.assertTrue(
            found, f"[{self.mode}] 断流 2.5 s 应进入 UNRELIABLE 并冻结航向"
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
