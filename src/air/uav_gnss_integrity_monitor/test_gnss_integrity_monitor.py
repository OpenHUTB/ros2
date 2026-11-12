"""纯计算层单元测试：不依赖 ROS、pytest、numpy 或仿真器。

运行方式（在本模块目录下）：
    python -m unittest -v test_gnss_integrity_monitor.py
    python -m pytest -v test_gnss_integrity_monitor.py   # 如已安装 pytest
"""

from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(MODULE_DIR))

from uav_gnss_integrity_monitor import (  # noqa: E402
    Config,
    Fix,
    Monitor,
    State,
    allowed_displacement,
    check_fix,
)

# 以文件路径加载 main.py，避免与仓库内其它模块的同名 main 冲突
_spec = importlib.util.spec_from_file_location(
    "gnss_replay_main", MODULE_DIR / "main.py"
)
replay_main = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = replay_main  # dataclass 需要能查到所在模块
_spec.loader.exec_module(replay_main)


def good(t: float, x: float = 0.0, y: float = 0.0, z: float = 5.0,
         fix_type: int = 3, num_sats: int = 12,
         accuracy: float = 0.4) -> Fix:
    return Fix(t, x, y, z, fix_type, num_sats, accuracy)


class TestConfigAndCheck(unittest.TestCase):
    def test_config_rejects_invalid_values(self):
        with self.assertRaises(ValueError):
            Config(max_speed=0.0)
        with self.assertRaises(ValueError):
            Config(recovery_streak=0)
        with self.assertRaises(ValueError):
            Config(stale_timeout=-1.0)

    def test_config_rejects_nan_and_inf(self):
        """NaN 与任何数比较都为 False，能绕过 `v <= 0` 检查，必须在构造时拒绝。

        若放行 max_speed=NaN，allowed_displacement 返回 NaN，`位移 > NaN`
        恒为 False，位置跳变判据被静默关闭；accuracy_limit=Inf 则直接
        把精度检查关掉。
        """
        fields = (
            "max_speed", "max_vertical_speed", "position_margin",
            "accuracy_limit", "stale_timeout", "max_extrapolate_dt",
            "reacquisition_radius",
        )
        for field in fields:
            for bad in (float("nan"), float("inf"), float("-inf")):
                with self.assertRaises(ValueError, msg=f"{field}={bad}"):
                    Config(**{field: bad})

    def test_nan_threshold_would_be_silently_accepted(self):
        """反证：不挡 NaN 的话，跳变判据会失效。"""
        nan = float("nan")
        self.assertFalse(nan <= 0)          # 旧检查漏掉 NaN 的原因
        self.assertFalse(25.0 > (nan * 0.1 + 1.5))  # 跳变判据被静默关闭

    def test_allowed_displacement_is_capped(self):
        cfg = Config(max_speed=12.0, max_extrapolate_dt=0.5,
                     position_margin=1.5)
        self.assertAlmostEqual(allowed_displacement(0.1, cfg)[0], 2.7)
        # 长时间间隔不能把跳变阈值无限放大
        self.assertEqual(
            allowed_displacement(10.0, cfg)[0],
            allowed_displacement(0.5, cfg)[0],
        )

    def test_quality_checks(self):
        cfg = Config()
        self.assertEqual(check_fix(None, good(0.0), cfg)[0], True)
        self.assertEqual(check_fix(None, good(0.0, fix_type=1), cfg)[1],
                         "fix_type")
        self.assertEqual(check_fix(None, good(0.0, num_sats=3), cfg)[1],
                         "low_sats")
        self.assertEqual(check_fix(None, good(0.0, accuracy=9.0), cfg)[1],
                         "accuracy")
        self.assertEqual(
            check_fix(None, Fix(0.0, float("nan"), 0.0, 5.0), cfg)[1],
            "non_finite",
        )

    def test_first_fix_initialises_tracking(self):
        monitor = Monitor()
        out = monitor.update(good(0.0, x=1.0))
        self.assertEqual(out.state, State.TRACKING)
        self.assertTrue(out.valid)
        self.assertEqual(out.x, 1.0)
        self.assertEqual(out.action, "track")

    def test_normal_motion_stays_tracking(self):
        monitor = Monitor()
        for i in range(10):
            out = monitor.update(good(i * 0.1, x=0.8 * i * 0.1))
        self.assertEqual(out.state, State.TRACKING)
        self.assertTrue(out.valid)
        self.assertEqual(out.reason, "ok")

    def test_position_jump_rejected(self):
        ok, reason = check_fix(good(0.0), good(0.1, x=25.0), Config())
        self.assertFalse(ok)
        self.assertEqual(reason, "position_jump")

    def test_small_displacement_accepted(self):
        ok, reason = check_fix(good(0.0), good(0.1, x=0.5), Config())
        self.assertTrue(ok)
        self.assertEqual(reason, "ok")

    def test_altitude_jump_rejected(self):
        ok, reason = check_fix(good(0.0), good(0.1, z=20.0), Config())
        self.assertFalse(ok)
        self.assertEqual(reason, "altitude_jump")

    def test_time_reversal_rejected(self):
        ok, reason = check_fix(good(1.0), good(0.5), Config())
        self.assertFalse(ok)
        self.assertEqual(reason, "time_reversal")

    def test_stale_rejected(self):
        ok, reason = check_fix(good(0.0), good(2.0), Config())
        self.assertFalse(ok)
        self.assertEqual(reason, "stale")

    def test_zero_dt_zero_displacement_accepted(self):
        ok, reason = check_fix(good(1.0, x=3.0), good(1.0, x=3.0), Config())
        self.assertTrue(ok)


class TestStateMachine(unittest.TestCase):
    def test_suspect_debounce_single_bad_frame(self):
        monitor = Monitor(Config(suspect_limit=2))
        monitor.update(good(0.0))
        out = monitor.update(good(0.1, x=25.0))
        self.assertEqual(out.state, State.SUSPECT)
        self.assertFalse(out.valid)
        # 单帧坏点后立刻恢复，不应进入 HOLD
        out = monitor.update(good(0.2, x=0.1))
        self.assertEqual(out.state, State.TRACKING)
        self.assertTrue(out.valid)

    def test_two_bad_frames_enter_hold(self):
        monitor = Monitor(Config(suspect_limit=2))
        monitor.update(good(0.0))
        monitor.update(good(0.1, x=25.0))
        out = monitor.update(good(0.2, x=25.0))
        self.assertEqual(out.state, State.HOLD)
        self.assertEqual(out.action, "hover")
        self.assertEqual(out.consecutive_bad, 2)

    def test_hold_freezes_last_good_pose(self):
        monitor = Monitor(Config(suspect_limit=1))
        monitor.update(good(0.0, x=2.0))
        out = monitor.update(good(0.1, x=25.0))
        self.assertEqual(out.state, State.HOLD)
        self.assertEqual(out.x, 2.0)
        for i in range(2, 6):
            out = monitor.update(good(i * 0.1, x=2.0 + 0.01 * i))
            self.assertEqual(out.x, 2.0)
            self.assertFalse(out.valid)

    def test_hold_recovery_requires_full_streak(self):
        cfg = Config(suspect_limit=1, recovery_streak=3,
                     reacquisition_radius=5.0)
        monitor = Monitor(cfg)
        monitor.update(good(0.0, x=2.0))
        monitor.update(good(0.1, x=25.0))
        for i in (2, 3):
            out = monitor.update(good(i * 0.1, x=2.0 + 0.01 * i))
            self.assertEqual(out.state, State.HOLD)
        out = monitor.update(good(0.4, x=2.04))
        self.assertEqual(out.state, State.TRACKING)
        self.assertEqual(out.reason, "reacquired")
        self.assertTrue(out.valid)

    def test_hold_rejects_far_reacquisition(self):
        cfg = Config(suspect_limit=1, recovery_streak=2,
                     reacquisition_radius=1.0)
        monitor = Monitor(cfg)
        monitor.update(good(0.0, x=0.0))
        monitor.update(good(0.1, x=25.0))
        for i in range(2, 6):
            out = monitor.update(good(i * 0.1, x=10.0))
        self.assertEqual(out.state, State.HOLD)
        self.assertEqual(out.reason, "far_from_hold_pose")

    def test_stream_gap_forces_hold(self):
        monitor = Monitor()
        monitor.update(good(0.0, x=1.0))
        out = monitor.update(good(2.0, x=1.0))
        self.assertEqual(out.state, State.HOLD)
        self.assertEqual(out.reason, "stream_gap")
        self.assertEqual(out.x, 1.0)

    def test_watchdog_timeout_forces_hold(self):
        monitor = Monitor()
        monitor.update(good(0.0, x=1.0))
        out = monitor.note_timeout()
        self.assertIsNotNone(out)
        self.assertEqual(out.state, State.HOLD)
        self.assertEqual(out.reason, "stream_timeout")
        self.assertIsNone(monitor.note_timeout())

    def test_reset_clears_state(self):
        monitor = Monitor()
        monitor.update(good(0.0, x=1.0))
        monitor.update(good(0.1, x=25.0))
        monitor.reset()
        self.assertEqual(monitor.state, State.INIT)
        self.assertIsNone(monitor.last_good)
        out = monitor.update(good(0.0, x=7.0))
        self.assertEqual(out.state, State.TRACKING)
        self.assertEqual(out.x, 7.0)

    def test_out_of_order_frame_rejected(self):
        """坏帧之后紧跟一帧延迟到达的旧数据，不能把门控拉回 TRACKING。

        旧帧的时间戳比 last_good 只大一点点、位移几乎为 0，旧实现下会被判成
        合法帧；判据必须对时间严格单调递增。
        """
        monitor = Monitor(Config(suspect_limit=2))
        monitor.update(good(1.00, x=0.0))          # 可信帧
        out = monitor.update(good(1.10, x=25.0))   # 跳变，进入 SUSPECT
        self.assertEqual(out.state, State.SUSPECT)
        self.assertEqual(out.consecutive_bad, 1)

        out = monitor.update(good(1.05, x=0.01))   # 乱序旧帧
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "out_of_order")
        self.assertEqual(out.state, State.SUSPECT)   # 没有被洗白
        self.assertEqual(out.x, 0.0)                 # 仍冻结在可信位姿
        self.assertEqual(out.consecutive_bad, 1)     # 旧帧不推进坏帧计数
        self.assertEqual(monitor.last_good.t, 1.00)

    def test_out_of_order_frame_does_not_trigger_hold(self):
        """轻微乱序不该被当成坏数据：连续旧帧只丢弃，不累计到 HOLD。"""
        monitor = Monitor(Config(suspect_limit=2))
        monitor.update(good(1.0, x=0.0))
        monitor.update(good(1.2, x=0.0))
        for t in (1.1, 1.15, 1.05, 0.9):
            out = monitor.update(good(t, x=0.0))
            self.assertEqual(out.reason, "out_of_order")
            self.assertFalse(out.valid)
        self.assertEqual(out.state, State.TRACKING)
        self.assertEqual(out.consecutive_bad, 0)
        # 新帧到达后照常跟踪
        out = monitor.update(good(1.3, x=0.0))
        self.assertEqual(out.state, State.TRACKING)
        self.assertTrue(out.valid)

    def test_monotonic_time_still_tracks(self):
        """加了单调性检查之后，正常递增的时间戳不受影响。"""
        monitor = Monitor()
        for i in range(5):
            out = monitor.update(good(i * 0.1, x=0.5 * i * 0.1))
        self.assertEqual(out.state, State.TRACKING)
        self.assertTrue(out.valid)


class TestOfflineReplay(unittest.TestCase):
    def test_replay_bounds_output_error(self):
        rows, events, metrics = replay_main.run_replay(Config())
        self.assertEqual(len(rows), metrics["samples"])
        self.assertGreater(metrics["raw_max_abs_error_m"], 20.0)
        self.assertLess(metrics["output_max_abs_error_m"], 3.0)
        self.assertLess(
            metrics["output_max_abs_error_m"],
            metrics["raw_max_abs_error_m"] / 5.0,
        )

    def test_replay_recovers_after_all_faults(self):
        rows, events, metrics = replay_main.run_replay(Config())
        self.assertGreaterEqual(metrics["recovery_count"], 3)
        self.assertEqual(rows[-1]["state"], "TRACKING")
        self.assertTrue(rows[-1]["valid"])
        reasons = {r for _, _, r in events}
        self.assertIn("stream_gap", reasons)
        self.assertIn("reacquired", reasons)
        self.assertTrue(all(math.isfinite(r["output_x"]) for r in rows))

    def test_svg_and_csv_output(self):
        rows, _, _ = replay_main.run_replay(Config())
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            replay_main.write_csv(out / "replay.csv", rows)
            replay_main.write_svg(out / "replay.svg", rows)
            text = (out / "replay.svg").read_text(encoding="utf-8")
            self.assertTrue(text.startswith("<?xml"))
            self.assertIn("<polyline", text)
            self.assertIn("</svg>", text)
            self.assertGreater(len((out / "replay.csv").read_text()), 100)


if __name__ == "__main__":
    unittest.main(verbosity=2)
