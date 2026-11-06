#!/usr/bin/env python3
"""uav_motor_thrust_imbalance_monitor 的单元测试（纯标准库，不需要 ROS）。

覆盖：
  * 线性指令→转速映射与 clamp_commands 封顶；
  * 基线学习（INIT → MONITORING 的门槛与冻结值）；
  * RPM 跟踪误差判据：warn/emg 两级、连帧升级、恢复；
  * 指令偏移判据：只在近悬停窗口生效、基线未就绪不判；
  * 补偿输出：IMBALANCE 抬升衰减电机指令并封顶，EMERGENCY 归零；
  * 非有限值 / 维度不符 / 乱序 / 断流 / 看门狗超时；
  * 离线回放产物（CSV/SVG）可写出且行数一致。
"""

from __future__ import annotations

import csv
import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from uav_motor_thrust_imbalance_monitor import (  # noqa: E402
    Config,
    MotorImbalanceMonitor,
    MotorSample,
    MotorState,
    clamp_commands,
    expected_rpm,
)

CFG = Config()
HOVER = 0.5
RPM_MAX = CFG.rpm_max


def hover_sample(t: float, eff: float = 1.0, dev0: float = 0.0,
                 cmd2: float = None) -> MotorSample:
    """悬停遥测帧：四路指令基线值 + 0 号电机附加偏移 + 2 号电机效率。"""

    cmds = [HOVER, HOVER, HOVER if cmd2 is None else cmd2, HOVER]
    cmds[0] += dev0
    rpm = []
    for i, c in enumerate(cmds):
        e = eff if i == 2 else 1.0
        rpm.append(expected_rpm(c, CFG) * e)
    return MotorSample(t=t, commands=cmds, rpm=rpm)


def learn_baseline(m: MotorImbalanceMonitor, n: int = 30) -> None:
    # 时间轴从 0 开始、步长 0.02 s：学完后 last_seen_t≈0.58 s，
    # 测试用例从 t≥1.0 s 续接，避免被判「乱序」。
    for k in range(n):
        m.update(hover_sample(k * 0.02))


class MappingTest(unittest.TestCase):
    def test_expected_rpm_linear(self):
        self.assertAlmostEqual(expected_rpm(0.0, CFG), 0.0)
        self.assertAlmostEqual(expected_rpm(1.0, CFG), RPM_MAX)
        self.assertAlmostEqual(expected_rpm(0.5, CFG), RPM_MAX / 2)
        self.assertEqual(expected_rpm(-0.2, CFG), 0.0)  # 负指令按 0 处理

    def test_clamp_commands_caps_and_floors(self):
        out = clamp_commands([0.5, 0.9, 0.2, 0.5], [0.4, 0.2, -0.5, 0.0], CFG)
        self.assertEqual(out, [0.9, 1.0, 0.0, 0.5])


class BaselineTest(unittest.TestCase):
    def test_init_then_monitoring_after_baseline_frames(self):
        m = MotorImbalanceMonitor(Config())
        states = [m.update(hover_sample(k * 0.1)).state
                  for k in range(CFG.baseline_frames + 2)]
        self.assertEqual(states[0], MotorState.INIT)
        self.assertEqual(states[-1], MotorState.MONITORING)
        # 基线应精确等于悬停指令
        self.assertEqual(m._baseline, [HOVER] * 4)

    def test_baseline_frozen_after_learning(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        b = list(m._baseline)
        for k in range(20):   # 后续帧即使指令漂移也不再更新基线
            m.update(hover_sample(10.0 + k * 0.1, dev0=0.1))
        self.assertEqual(m._baseline, b)


class TrackingErrorTest(unittest.TestCase):
    def test_warn_then_emergency_then_recover(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        # 效率 0.85 → 稳态跟踪误差 15%；EMA 从 0 爬升需要几帧，
        # 连帧 3 次后 IMBALANCE（回放实测 ~1.8 s 检测延迟，属设计特性）
        out = None
        for k in range(6):
            out = m.update(hover_sample(1.0 + k * 0.1, eff=0.85))
        self.assertEqual(out.state, MotorState.IMBALANCE)
        self.assertEqual(out.worst_motor, 2)
        self.assertFalse(out.emergency_land)
        self.assertGreater(out.compensation[2], 0.0)

        # 效率 0.72 → 稳态 28%（> emg 门限 25%）：EMA 爬升 + 5 连帧后
        # EMERGENCY，并触发紧急降落建议
        for k in range(14):
            out = m.update(hover_sample(2.0 + k * 0.1, eff=0.72))
        self.assertEqual(out.state, MotorState.EMERGENCY)
        self.assertTrue(out.emergency_land)
        self.assertEqual(out.compensation, [0.0] * 4)   # 紧急状态下不再补偿

        # 恢复：EMA 衰减到门限以下，随后 recovery_streak 帧恢复
        ok = 0
        state_seq = []
        for k in range(25):
            out = m.update(hover_sample(4.0 + k * 0.1, eff=1.0))
            state_seq.append(out.state)
            if out.state == MotorState.MONITORING:
                ok += 1
        self.assertTrue(ok > 0, f"应恢复 MONITORING：{state_seq}")

    def test_ema_smooths_single_spike(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        out = m.update(hover_sample(1.0, eff=0.5))       # 单帧 50% 误差
        self.assertEqual(out.state, MotorState.MONITORING)
        self.assertGreater(out.track_err_pct[2], 0.0)    # 信号被记录
        self.assertLess(out.track_err_pct[2], 50.0)      # 但被 EMA 平滑


class CommandDeviationTest(unittest.TestCase):
    def test_prop_loss_detected_by_cmd_deviation(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        # 桨损：转速正常（eff=1），但 0 号电机必须多拉 24% 悬停指令；
        # EMA 爬坡 + 3 连帧，~6 帧内到 IMBALANCE
        out = None
        for k in range(8):
            out = m.update(hover_sample(1.0 + k * 0.1, dev0=0.12))
        self.assertEqual(out.state, MotorState.IMBALANCE)
        self.assertEqual(out.worst_motor, 0)
        self.assertLess(out.track_err_pct[0], 1.0)      # 转速跟踪正常
        self.assertFalse(out.emergency_land)

    def test_deviation_not_judged_far_from_hover(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        # 大机动：全部指令抬到 0.85（远离悬停窗口），即使 0 号偏高也不判
        t = 1.0
        for k in range(6):
            cmds = [0.97, 0.85, 0.85, 0.85]
            rpm = [expected_rpm(c, CFG) for c in cmds]
            out = m.update(MotorSample(t=t, commands=cmds, rpm=rpm))
            t += 0.1
        self.assertEqual(out.state, MotorState.MONITORING)

    def test_no_rpm_falls_back_to_deviation_only(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        for k in range(4):
            out = m.update(MotorSample(
                t=1.0 + k * 0.1, commands=[HOVER, HOVER, HOVER, HOVER],
                rpm=None,
            ))
        self.assertEqual(out.state, MotorState.MONITORING)
        self.assertFalse(out.rpm_available)
        out = None
        for k in range(8):
            out = m.update(MotorSample(
                t=2.0 + k * 0.1,
                commands=[HOVER + 0.12, HOVER, HOVER, HOVER], rpm=None,
            ))
        self.assertEqual(out.state, MotorState.IMBALANCE)


class CompensationTest(unittest.TestCase):
    def test_compensation_capped_and_saturates_command(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        for k in range(4):
            out = m.update(hover_sample(1.0 + k * 0.1, eff=0.85))
        self.assertLessEqual(out.compensation[2], CFG.comp_limit)
        merged = clamp_commands(
            [HOVER] * 4, out.compensation, CFG)
        self.assertLessEqual(merged[2], CFG.cmd_max)
        # 指令已经接近饱和时，合并后不得越界
        merged2 = clamp_commands([0.95, 0.9, 0.95, 0.9],
                                 [0.15, 0.0, 0.15, 0.0], CFG)
        self.assertEqual(merged2, [1.0, 0.9, 1.0, 0.9])


class RobustnessTest(unittest.TestCase):
    def test_non_finite_rejected(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        out = m.update(MotorSample(1.0, [0.5, float("nan"), 0.5, 0.5],
                                   [4000.0] * 4))
        self.assertFalse(out.healthy)
        self.assertEqual(out.reason, "non_finite")

    def test_wrong_length_rejected(self):
        m = MotorImbalanceMonitor(Config())
        out = m.update(MotorSample(0.0, [0.5, 0.5]))
        self.assertEqual(out.reason, "non_finite")
        out = m.update(MotorSample(0.1, [0.5] * 4, rpm=[1.0, 2.0]))
        self.assertEqual(out.reason, "non_finite")

    def test_out_of_order_ignored(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        out = m.update(hover_sample(5.0))
        out2 = m.update(hover_sample(4.9))              # 时间倒退
        self.assertEqual(out2.reason, "out_of_order")

    def test_stream_gap_after_dropout(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        m.update(hover_sample(1.0))
        out = m.update(hover_sample(3.0))               # 2 s 无遥测
        self.assertEqual(out.reason, "stream_gap")
        self.assertFalse(out.healthy)

    def test_watchdog_timeout_degrades_but_caps_at_imbalance(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        m.update(hover_sample(1.0))
        out = m.note_timeout(1.0 + CFG.stale_timeout + 0.1)
        self.assertIsNotNone(out)
        self.assertEqual(out.reason, "stream_timeout")
        # 持续超时：连帧降级，但**封顶在 IMBALANCE**——遥测丢失不能等同
        # 动力失效，不允许仅凭断流触发紧急降落（安全设计，见 note_timeout）
        for k in range(CFG.emergency_limit + 4):
            out = m.note_timeout(2.5 + k * 0.2)
            if out is None:
                continue
        self.assertEqual(out.state, MotorState.IMBALANCE)
        self.assertFalse(out.emergency_land)

    def test_watchdog_silent_before_timeout(self):
        m = MotorImbalanceMonitor(Config())
        learn_baseline(m)
        m.update(hover_sample(1.0))
        self.assertIsNone(m.note_timeout(1.0 + 0.5))    # 未超时


class ReplayOutputTest(unittest.TestCase):
    def test_plot_outputs_are_written(self):
        # main.py 以脚本或包两种方式都可导入
        spec = importlib.util.spec_from_file_location(
            "motor_replay_main",
            Path(__file__).resolve().parent / "main.py",
        )
        replay_main = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(replay_main)

        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            _, rows, events, _ = replay_main.run_replay(Config())
            replay_main.write_csv(out_dir / "offline_replay.csv", rows)
            replay_main.write_svg(out_dir / "offline_replay.svg", rows)
            with (out_dir / "offline_replay.csv").open(encoding="utf-8") as fh:
                self.assertEqual(len(list(csv.DictReader(fh))), len(rows))
            svg = (out_dir / "offline_replay.svg").read_text(encoding="utf-8")
            self.assertIn("<svg", svg)
            self.assertIn("polyline", svg)
            self.assertIn("</svg>", svg)
        # 回放应完整走过三个关键状态
        states = {e[1] for e in events}
        self.assertIn(MotorState.IMBALANCE, states)
        self.assertIn(MotorState.EMERGENCY, states)
        self.assertIn(MotorState.MONITORING, states)


if __name__ == "__main__":
    unittest.main(verbosity=2)
