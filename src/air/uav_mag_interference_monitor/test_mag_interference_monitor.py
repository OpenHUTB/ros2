"""uav_mag_interference_monitor 的单元测试（纯标准库，无需 ROS 2）。

运行：
    python -m unittest -v test_mag_interference_monitor.py
"""

from __future__ import annotations

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from uav_mag_interference_monitor import (  # noqa: E402
    Config,
    FlightSample,
    HeadingMonitor,
    HeadingState,
    MagSample,
    PreflightState,
    fit_circle,
    run_preflight,
    wrap_angle,
)

H = 22.0     # 水平地磁 (µT)
V = -42.0    # 垂直地磁 (µT)
BIAS = (4.0, -2.5, 1.2)


def circle_samples(n: int = 64, offset=(0.0, 0.0, 0.0), bias=BIAS):
    """整机旋转一周的磁场样本（x 前、y 右，NED/FRD 约定）。"""

    out = []
    for i in range(n):
        yaw = 2.0 * math.pi * i / n
        mx = H * math.cos(yaw) + bias[0] + offset[0]
        my = H * math.sin(yaw) + bias[1] + offset[1]
        mz = V + bias[2] + offset[2]
        out.append(MagSample(i * 0.05, mx, my, mz))
    return out


def flight(yaw_deg: float, cross: float = 0.0, along: float = 0.0,
           t: float = 0.0, speed: float = 5.0, course_deg: float | None = None,
           bias=BIAS) -> FlightSample:
    """按给定真实航向与干扰量构造一帧观测。"""

    yaw = math.radians(yaw_deg)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rx, ry = -fy, fx
    mx = H * fx + cross * rx + along * fx + bias[0]
    my = H * fy + cross * ry + along * fy + bias[1]
    mz = V + bias[2]
    course = None if course_deg is None else math.radians(course_deg)
    return FlightSample(t, mx, my, mz, speed=speed, course=course)


class TestCircleFit(unittest.TestCase):
    def test_recovers_horizontal_bias(self):
        fit = fit_circle(circle_samples())
        for got, want in zip(fit.bias, BIAS[:2]):
            self.assertAlmostEqual(got, want, places=6)

    def test_radius_matches_horizontal_field(self):
        fit = fit_circle(circle_samples())
        self.assertAlmostEqual(fit.radius, H, places=6)

    def test_residual_is_small_on_clean_data(self):
        fit = fit_circle(circle_samples())
        self.assertLess(fit.residual_rms, 1e-6)

    def test_vertical_mean_tracks_earth_field(self):
        fit = fit_circle(circle_samples())
        self.assertAlmostEqual(fit.z_mean, V + BIAS[2], places=6)

    def test_counts_samples(self):
        self.assertEqual(fit_circle(circle_samples(50)).n_samples, 50)

    def test_rejects_too_few_samples(self):
        with self.assertRaises(ValueError):
            fit_circle(circle_samples(7))

    def test_ignores_non_finite_samples(self):
        samples = circle_samples(40) + [MagSample(9.9, float("nan"), 1.0, 2.0)]
        fit = fit_circle(samples)
        self.assertEqual(fit.n_samples, 40)
        self.assertTrue(all(math.isfinite(v) for v in fit.bias))


class TestPreflight(unittest.TestCase):
    def test_pass_when_motor_off(self):
        res = run_preflight(circle_samples(), circle_samples())
        self.assertEqual(res.state, PreflightState.PASS)
        self.assertEqual(res.reason, "ok")
        self.assertLess(res.bias_shift_pct, 1.0)

    def test_fail_on_large_motor_offset(self):
        spin = circle_samples(offset=(5.6, -3.2, 1.6))
        res = run_preflight(circle_samples(), spin)
        self.assertEqual(res.state, PreflightState.FAIL)
        self.assertEqual(res.reason, "bias_shift_exceeds_limit")
        self.assertGreater(res.bias_shift_pct, 5.0)

    def test_warn_in_middle_band(self):
        # 偏移约 4% 地磁水平分量：超过 warn 门限但未到失败门限
        scale = 0.04 * H
        spin = circle_samples(offset=(scale, 0.0, 0.0))
        res = run_preflight(circle_samples(), spin)
        self.assertEqual(res.state, PreflightState.WARN)
        self.assertEqual(res.reason, "mild_interference")

    def test_reports_shift_in_microtesla(self):
        spin = circle_samples(offset=(5.6, -3.2, 1.6))
        res = run_preflight(circle_samples(), spin)
        self.assertAlmostEqual(res.bias_shift_ut, math.dist(
            (0.0, 0.0, 0.0), (5.6, -3.2, 1.6)), places=5)

    def test_vertical_offset_alone_can_fail(self):
        # 水平偏差很小但垂直方向被抬高 10 µT，仍应判为失败
        spin = circle_samples(offset=(0.2, 0.0, 10.0))
        res = run_preflight(circle_samples(), spin)
        self.assertEqual(res.state, PreflightState.FAIL)

    def test_rejects_insufficient_idle_samples(self):
        with self.assertRaises(ValueError):
            run_preflight(circle_samples(10), circle_samples())

    def test_rejects_insufficient_spin_samples(self):
        with self.assertRaises(ValueError):
            run_preflight(circle_samples(), circle_samples(10))


class TestHeadingMath(unittest.TestCase):
    def test_wrap_angle_range(self):
        for a in (-7.0, -math.pi - 0.1, 0.0, math.pi + 0.1, 9.0):
            w = wrap_angle(a)
            self.assertGreater(w, -math.pi)
            self.assertLessEqual(w, math.pi)

    def test_wrap_angle_identity(self):
        self.assertAlmostEqual(wrap_angle(0.3), 0.3, places=12)


class TestHeadingMonitor(unittest.TestCase):
    def monitor(self, **kw) -> HeadingMonitor:
        return HeadingMonitor(BIAS, Config(**kw), expected_radius=H)

    def test_first_clean_frame_is_trusted(self):
        out = self.monitor().update(flight(45.0, course_deg=45.0))
        self.assertEqual(out.state, HeadingState.TRUSTED)
        self.assertTrue(out.valid)
        self.assertEqual(out.heading_source, "mag")
        self.assertAlmostEqual(math.degrees(out.heading), 45.0, places=6)

    def test_heading_error_reported(self):
        s = flight(45.0, cross=8.0, course_deg=45.0)
        out = self.monitor().update(s)
        expected = math.degrees(math.atan2(8.0, H))
        self.assertAlmostEqual(out.heading_error_deg, expected, places=5)

    def test_no_course_no_error(self):
        out = self.monitor().update(flight(45.0))
        self.assertTrue(math.isnan(out.heading_error_deg))

    def test_degraded_after_degraded_limit(self):
        m = self.monitor()
        states = [
            m.update(flight(45.0, cross=8.0, course_deg=45.0, t=i * 0.1)).state
            for i in range(m.cfg.degraded_limit + 1)
        ]
        self.assertEqual(states[0], HeadingState.INIT)   # 还没建立可信航向
        self.assertEqual(states[-1], HeadingState.DEGRADED)

    def test_unreliable_after_unreliable_limit(self):
        m = self.monitor()
        out = None
        for i in range(m.cfg.unreliable_limit + 1):
            out = m.update(flight(45.0, cross=18.0, course_deg=45.0, t=i * 0.1))
        self.assertEqual(out.state, HeadingState.UNRELIABLE)
        self.assertEqual(out.heading_source, "gnss_course")
        self.assertTrue(out.valid)
        self.assertAlmostEqual(math.degrees(out.heading), 45.0, places=6)

    def test_unreliable_without_course_holds_last_trusted(self):
        m = self.monitor()
        m.update(flight(45.0, course_deg=45.0, t=0.0))
        out = None
        for i in range(1, m.cfg.unreliable_limit + 2):
            out = m.update(flight(90.0, cross=18.0, t=i * 0.1))
        self.assertEqual(out.state, HeadingState.UNRELIABLE)
        self.assertEqual(out.heading_source, "last_trusted")
        self.assertFalse(out.valid)
        self.assertAlmostEqual(math.degrees(out.heading), 45.0, places=6)

    def test_recovery_after_streak(self):
        m = self.monitor()
        for i in range(m.cfg.unreliable_limit + 1):
            m.update(flight(45.0, cross=18.0, course_deg=45.0, t=i * 0.1))
        out = None
        for i in range(m.cfg.recovery_streak + 2):
            out = m.update(flight(45.0, course_deg=45.0, t=10.0 + i * 0.1))
        self.assertEqual(out.state, HeadingState.TRUSTED)
        self.assertEqual(out.heading_source, "mag")

    def test_norm_deviation_detected(self):
        m = self.monitor()
        m.update(flight(45.0, course_deg=45.0, t=0.0))
        out = None
        for i in range(1, m.cfg.degraded_limit + 1):
            out = m.update(flight(45.0, along=18.0, course_deg=45.0, t=i * 0.1))
        self.assertEqual(out.state, HeadingState.DEGRADED)
        self.assertIn("field_norm_out_of_range", out.reason)
        self.assertGreater(out.field_norm_pct, 100.0 + m.cfg.norm_tolerance_pct)

    def test_non_finite_input_is_rejected(self):
        m = self.monitor()
        # 没有 GNSS 航迹向可兜底时必须标记为不可用
        out = m.update(FlightSample(0.0, float("nan"), 0.0, 0.0, 0.0, None))
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "non_finite")
        # 有可信航迹向时允许改用航迹向，但仍需标记数据本身异常
        out2 = m.update(FlightSample(0.1, float("nan"), 0.0, 0.0, 5.0, 0.0))
        self.assertEqual(out2.heading_source, "gnss_course")
        self.assertEqual(out2.reason, "non_finite")

    def test_stale_gap_flagged(self):
        m = self.monitor()
        m.update(flight(45.0, course_deg=45.0, t=0.0))
        out = m.update(flight(45.0, course_deg=45.0, t=m.cfg.stale_timeout + 0.5))
        self.assertIn("stale", out.reason)

    def test_low_speed_does_not_compare_course(self):
        # 悬停时地速过低，航迹向不可信，应由上游传 course=None
        out = self.monitor().update(flight(45.0, speed=0.3))
        self.assertTrue(math.isnan(out.heading_error_deg))
        self.assertEqual(out.state, HeadingState.TRUSTED)

    def test_timeout_without_data_returns_none(self):
        self.assertIsNone(self.monitor().note_timeout(1.0))

    def test_timeout_escalates_to_unreliable(self):
        m = self.monitor()
        m.update(flight(45.0, course_deg=45.0, t=0.0))
        out = None
        for i in range(m.cfg.unreliable_limit + 1):
            out = m.note_timeout(5.0 + i * 0.1, course=math.radians(45.0))
        self.assertEqual(out.state, HeadingState.UNRELIABLE)
        self.assertEqual(out.reason, "stream_timeout")
        self.assertEqual(out.heading_source, "gnss_course")


class TestConfig(unittest.TestCase):
    def test_rejects_warn_above_fail(self):
        with self.assertRaises(ValueError):
            Config(warn_bias_shift_pct=6.0, max_bias_shift_pct=5.0)

    def test_rejects_inverted_limits(self):
        with self.assertRaises(ValueError):
            Config(degraded_limit=9, unreliable_limit=4)

    def test_rejects_non_positive_thresholds(self):
        with self.assertRaises(ValueError):
            Config(max_heading_error_deg=0.0)

    def test_rejects_small_preflight_window(self):
        with self.assertRaises(ValueError):
            Config(min_preflight_samples=4)


class TestReplayEntryPoint(unittest.TestCase):
    def test_replay_runs_end_to_end(self):
        import main as replay_main

        preflight, rows, events, metrics = replay_main.run_replay(Config())
        self.assertEqual(preflight["ok"].state, PreflightState.PASS)
        self.assertEqual(preflight["bad"].state, PreflightState.FAIL)
        self.assertGreater(len(rows), 300)
        self.assertGreater(len(events), 2)
        self.assertGreater(metrics["heading_source_switches"], 0)
        self.assertGreaterEqual(metrics["recovery_count"], 1)
        # 门控后的航向偏差不应大于未门控的磁航向偏差
        self.assertLessEqual(
            metrics["output_max_error_deg"], metrics["mag_max_error_deg"]
        )

    def test_plot_outputs_are_written(self):
        import main as replay_main

        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            _, rows, _, _ = replay_main.run_replay(Config())
            replay_main.write_csv(out_dir / "offline_replay.csv", rows)
            replay_main.write_svg(out_dir / "offline_replay.svg", rows)
            with (out_dir / "offline_replay.csv").open(encoding="utf-8") as fh:
                self.assertEqual(len(list(csv.DictReader(fh))), len(rows))
            svg = (out_dir / "offline_replay.svg").read_text(encoding="utf-8")
            self.assertIn("<svg", svg)
            self.assertIn("polyline", svg)
            self.assertIn("</svg>", svg)

    def test_preflight_csv_roundtrip(self):
        import main as replay_main

        samples = circle_samples(30)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "idle.csv"
            replay_main.write_mag_csv(path, samples)
            loaded = replay_main.load_mag_csv(path)
        self.assertEqual(len(loaded), 30)
        for got, want in zip(loaded, samples):
            self.assertAlmostEqual(got.mx, want.mx, places=2)
            self.assertAlmostEqual(got.mz, want.mz, places=2)

    def test_load_mag_csv_reports_bad_columns(self):
        import main as replay_main

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.csv"
            path.write_text("t,mx,my\n0,1,2\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                replay_main.load_mag_csv(path)


if __name__ == "__main__":
    unittest.main()
