# -*- coding: utf-8 -*-
"""imu_road_anomaly_detection 的可收集单元测试。

只用标准库 ``unittest``，不依赖 ``carla`` / ``numpy`` / ``Pillow``，
因此在本机任意 Python 3.9+ 环境都能直接跑：

    python -m unittest discover -s tests -v
    python -m pytest tests -q

覆盖点全部对应本模块修过的真实缺陷，作为回归测试：
1. 短窗 Goertzel 的整数 bin 取整把 1 Hz / 2 Hz 塌到同一个 bin；
2. 分辨率不足时不应输出频谱质心；
3. pothole 置信度必须随幅值严格单调递增，而不是撞在常数上；
4. 三类标签的判定规则；
5. high_pass_residual 真的用了 gx / gy（不能退化成 az - gz）；
6. gravity_align 对孤立毛刺的中位数鲁棒性；
7. imu_eval 的全局最小总时差匹配优于按 GT 顺序的贪心匹配；
8. self_test 的自检链路可用。
"""

from __future__ import annotations

import io
import math
import os
import sys
import unittest
from contextlib import redirect_stdout

# 模块目录本身不是包，测试文件在 tests/ 下，先把它加进 sys.path
MODULE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

import imu_anomaly  # noqa: E402
import imu_eval  # noqa: E402


def _cfg(**overrides):
    """返回一份 DEFAULTS 的副本，可按需覆盖若干阈值。"""
    cfg = dict(imu_anomaly.DEFAULTS)
    cfg.update(overrides)
    return cfg


def _sin(freq, fs, n, amp=1.0, phase=0.0):
    return [amp * math.sin(2 * math.pi * freq * i / fs + phase) for i in range(n)]


class GoertzelPowerTest(unittest.TestCase):
    """回归：短窗下 1 Hz / 2 Hz 不能塌到同一个 DFT bin。"""

    def test_short_window_keeps_fractional_bins_apart(self):
        fs, n = 100, 20          # 0.2 s 窗 → 分辨率 5 Hz
        sig = _sin(2.0, fs, n)   # 主频 2 Hz
        p1 = imu_anomaly._goertzel_power(sig, 1.0, fs)
        p2 = imu_anomaly._goertzel_power(sig, 2.0, fs)
        # 旧的整数 bin 版本把 1 Hz 与 2 Hz 都四舍五入到 k=0，两个功率完全相等；
        # 现在按小数频率直接求值，两者必须可区分且都非零。
        self.assertGreater(abs(p2 - p1), 1e-6,
                           "1 Hz 与 2 Hz 的功率仍然塌到同一个 bin")
        self.assertGreater(p1, 0.0)
        self.assertGreater(p2, 0.0)

    def test_power_peaks_at_true_frequency(self):
        fs, n = 100, 100         # 分辨率正好 1 Hz
        sig = _sin(10.0, fs, n)
        p_lo = imu_anomaly._goertzel_power(sig, 9.0, fs)
        p_mid = imu_anomaly._goertzel_power(sig, 10.0, fs)
        p_hi = imu_anomaly._goertzel_power(sig, 11.0, fs)
        self.assertGreater(p_mid, p_lo)
        self.assertGreater(p_mid, p_hi)

    def test_empty_window_is_zero(self):
        self.assertEqual(imu_anomaly._goertzel_power([], 5.0, 100.0), 0.0)


class SpectralCentroidTest(unittest.TestCase):
    """回归：窗长不足时不应硬报一个质心。"""

    def test_short_window_returns_none(self):
        # 10 个样本 @ 100 Hz → 分辨率 10 Hz，远粗于 2 Hz 的要求
        self.assertIsNone(
            imu_anomaly._spectral_centroid(_sin(5.0, 100, 10), 100.0))

    def test_long_window_returns_finite_positive(self):
        centroid = imu_anomaly._spectral_centroid(_sin(10.0, 100, 400), 100.0)
        self.assertIsNotNone(centroid)
        self.assertTrue(math.isfinite(centroid))
        self.assertGreater(centroid, 0.0)

    def test_empty_or_invalid_rate_returns_none(self):
        self.assertIsNone(imu_anomaly._spectral_centroid([], 100.0))
        self.assertIsNone(imu_anomaly._spectral_centroid([1.0, 2.0], 0.0))


class ClassifyTest(unittest.TestCase):
    """分类规则与 pothole 置信度的幅值单调性。"""

    def test_pothole_confidence_strictly_increases_with_amplitude(self):
        cfg = _cfg()
        amps = [5.1, 10.0, 100.0, 1000.0]
        confs = []
        for amp in amps:
            label, conf = imu_anomaly._classify(amp, 0.05, None, cfg)
            self.assertEqual(label, "pothole", "%s m/s² 应判为 pothole" % amp)
            confs.append(conf)
        for prev, cur in zip(confs, confs[1:]):
            self.assertGreater(cur, prev,
                               "置信度未随幅值严格递增：%r" % (confs,))
        self.assertGreater(len(set(confs)), 1, "置信度退化成了常数")
        for conf in confs:
            self.assertGreater(conf, 0.0)
            self.assertLessEqual(conf, 1.0)

    def test_three_labels(self):
        cfg = _cfg()
        # 强而短 → 坑洼
        self.assertEqual(imu_anomaly._classify(20.0, 0.05, None, cfg)[0],
                         "pothole")
        # 强而宽（≥ 0.12 s）→ 减速带
        self.assertEqual(imu_anomaly._classify(20.0, 0.30, None, cfg)[0],
                         "speed_bump")
        # 低幅且持续（≥ 0.30 s）→ 粗糙路面
        self.assertEqual(imu_anomaly._classify(2.0, 0.50, None, cfg)[0],
                         "rough_patch")
        # 低幅但短 → 仍归粗糙路面（弱不平），不应误判成坑洼
        self.assertEqual(imu_anomaly._classify(2.0, 0.10, None, cfg)[0],
                         "rough_patch")

    def test_confidence_within_unit_interval(self):
        cfg = _cfg()
        for amp, dur in [(1.0, 0.1), (5.0, 0.1), (80.0, 0.05),
                         (80.0, 0.5), (3.0, 2.0)]:
            _, conf = imu_anomaly._classify(amp, dur, 12.0, cfg,
                                            crest_factor=9.0,
                                            eff_duration=0.02)
            self.assertGreaterEqual(conf, 0.0)
            self.assertLessEqual(conf, 1.0)


class HighPassResidualTest(unittest.TestCase):
    """回归：重力投影必须真的用到 gx / gy，而不是 az - gz。"""

    def _samples_with_lateral_gravity(self, n=200, bump_index=100,
                                       bump=6.0, fs=100):
        samples = []
        for i in range(n):
            ax = 9.81                      # 重力全落在 x 轴
            if bump_index <= i < bump_index + 3:
                ax += bump                # 纵向冲击（横向重力分量上）
            samples.append(imu_anomaly.IMUSample(
                t=i / fs, ax=ax, ay=0.0, az=0.0))
        return samples

    def test_uses_lateral_gravity_component(self):
        samples = self._samples_with_lateral_gravity()
        gx, gy, gz = imu_anomaly.gravity_align(samples,
                                               baseline_frames=50)
        residual = imu_anomaly.high_pass_residual(samples, gx, gy, gz, 30,
                                                  200.0)
        peak = max(abs(v) for v in residual)
        # 只减 gz 的实现会得到一条近似 0 的残差，看不到这个冲击
        self.assertGreater(peak, 1.0,
                           "横向重力分量上的冲击没有被投影进残差（峰值 %.3f）"
                           % peak)
        # 平路区间应基本安静
        quiet = max(abs(v) for v in residual[:80])
        self.assertLess(quiet, 0.5)

    def test_degenerate_gravity_falls_back_to_z_axis(self):
        samples = [imu_anomaly.IMUSample(t=i / 100.0, ax=0.0, ay=0.0,
                                         az=9.81 + (8.0 if 50 <= i < 53 else 0.0))
                   for i in range(200)]
        residual = imu_anomaly.high_pass_residual(samples, 0.0, 0.0, 0.0, 30)
        self.assertGreater(max(abs(v) for v in residual), 1.0)


class GravityAlignTest(unittest.TestCase):
    """重力向量用中位数估计，必须扛得住单帧非物理毛刺。"""

    def test_median_ignores_isolated_glitch(self):
        samples = []
        for i in range(300):
            az = 9.81
            if i == 150:
                az = -12029.0           # CARLA 生成瞬间出现过的非物理读数
            samples.append(imu_anomaly.IMUSample(t=i / 100.0, ax=0.0,
                                                 ay=0.0, az=az))
        _, _, gz = imu_anomaly.gravity_align(samples, baseline_frames=50)
        self.assertAlmostEqual(gz, 9.81, places=3)

    def test_reject_glitches_reports_count(self):
        raw = [9.8] * 100
        raw[42] = 9000.0
        cleaned, replaced = imu_anomaly.reject_glitches(raw, limit=200.0)
        self.assertEqual(replaced, 1)
        self.assertAlmostEqual(cleaned[42], imu_anomaly._median(raw[:42] +
                                                                raw[43:]))
        self.assertEqual(len(cleaned), len(raw))


class GlobalMatchingTest(unittest.TestCase):
    """回归：全局最小总时差匹配不能被按 GT 顺序的贪心替代。"""

    def test_global_beats_greedy(self):
        # gt 0.3 s 最近的检测是 0.5 s（差 0.2），但 0.5 s 那个才是 gt 0.5 s 的
        # 正解；贪心会把 0.5 抢走、再用 0.0 凑（差 0.5），总代价 0.7；全局最优
        # 是 0.0 ↔ 0.3（0.3）+ 0.5 ↔ 0.5（0.0），总代价 0.3。
        gt = [{"t": 0.3, "type": "pothole"},
              {"t": 0.5, "type": "pothole"}]
        dets = [{"t_peak": 0.0, "label": "pothole"},
                {"t_peak": 0.5, "label": "pothole"}]
        matches = imu_eval._global_min_cost_matching(gt, dets, 0.5)
        self.assertEqual(len(matches), 2)
        total = sum(m[2] for m in matches)
        self.assertAlmostEqual(total, 0.3, places=6)
        pair = {round(m[0]["t"], 6): round(m[1]["t_peak"], 6) for m in matches}
        self.assertEqual(pair[0.3], 0.0)
        self.assertEqual(pair[0.5], 0.5)

    def test_tolerance_is_respected(self):
        gt = [{"t": 0.0, "type": "pothole"}]
        dets = [{"t_peak": 0.9, "label": "pothole"}]
        self.assertEqual(imu_eval._global_min_cost_matching(gt, dets, 0.5), [])

    def test_one_to_one_assignment(self):
        gt = [{"t": 0.0, "type": "pothole"},
              {"t": 1.0, "type": "speed_bump"}]
        dets = [{"t_peak": 0.02, "label": "pothole"}]
        matches = imu_eval._global_min_cost_matching(gt, dets, 0.5)
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0][0]["t"], 0.0)


class SelfTestTest(unittest.TestCase):
    """自检链路（合成信号 → 检测 → 分类）必须仍然通过。"""

    def test_self_test_returns_true(self):
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            ok = imu_anomaly.self_test()
        self.assertTrue(ok, buffer.getvalue())
        self.assertIn("PASS", buffer.getvalue())


if __name__ == "__main__":
    unittest.main()
