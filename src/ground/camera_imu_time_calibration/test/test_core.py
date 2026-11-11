#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核心算法单元测试（不需要 ROS，可直接 python3 -m unittest 或 pytest 运行）。

    cd src/ground/camera_imu_time_calibration
    python3 -m unittest discover -s test -v
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from camera_imu_time_calibration import core  # noqa: E402


class TestTrajectory(unittest.TestCase):
    def test_omega_shape_and_scale(self):
        """解析轨迹的形状正确，峰值在 1 rad/s 量级。"""
        self.assertEqual(core.omega(0.0).shape, (3,))
        self.assertEqual(core.omega(np.zeros(5)).shape, (5, 3))
        self.assertAlmostEqual(core.omega_peak(), 1.346, delta=0.02)

    def test_omega_is_bounded(self):
        """任意时刻角速度不超过 1.5 rad/s，保证仿真不会高速旋转。"""
        t = np.linspace(0.0, 20.0, 5000)
        self.assertLess(np.abs(core.omega(t)).max(), 1.5)


class TestSimulate(unittest.TestCase):
    def test_shapes_and_rates(self):
        """IMU 200 Hz、相机 30 Hz 的样本数与时长一致。"""
        sim = core.simulate(seed=0, duration=10.0, imu_rate=200.0, cam_rate=30.0)
        self.assertEqual(len(sim['t_imu']), 2001)
        self.assertEqual(len(sim['t_cam']), 300)
        self.assertEqual(sim['w_imu'].shape, (2001, 3))
        self.assertEqual(sim['w_cam'].shape, (300, 3))

    def test_rejects_bad_timestamp_mode(self):
        with self.assertRaises(ValueError):
            core.simulate(timestamp_at='end')

    def test_camera_is_lower_rate(self):
        """相机样本必须比 IMU 稀疏，这是本模块的前提。"""
        sim = core.simulate(seed=0, imu_rate=200.0, cam_rate=10.0)
        self.assertLess(len(sim['t_cam']) * 10, len(sim['t_imu']))


class TestEstimateOffset(unittest.TestCase):
    def test_recovers_zero_offset(self):
        _, res = core.trial(seed=0, offset=0.0)
        self.assertAlmostEqual(res['offset'], 0.0, delta=0.002)

    def test_recovers_positive_offset(self):
        _, res = core.trial(seed=0, offset=0.050)
        self.assertAlmostEqual(res['offset'], 0.050, delta=0.002)

    def test_recovers_negative_offset(self):
        _, res = core.trial(seed=0, offset=-0.080)
        self.assertAlmostEqual(res['offset'], -0.080, delta=0.002)

    def test_correlation_peak_is_high(self):
        _, res = core.trial(seed=0, offset=0.050)
        self.assertGreater(res['correlation'], 0.95)

    def test_immune_to_constant_gyro_bias(self):
        """常值零偏被去均值消掉，估计结果不应改变。"""
        _, a = core.trial(seed=0, offset=0.050, imu_bias=0.0)
        _, b = core.trial(seed=0, offset=0.050, imu_bias=0.500)
        self.assertAlmostEqual(a['offset'], b['offset'], delta=0.001)

    def test_offset_decoupled_from_scale_error(self):
        """尺度误差不应污染偏移估计，且能被独立估出来。"""
        _, a = core.trial(seed=0, offset=0.050, cam_gain=1.0)
        _, b = core.trial(seed=0, offset=0.050, cam_gain=1.2)
        self.assertAlmostEqual(a['offset'], b['offset'], delta=0.001)
        self.assertAlmostEqual(b['scale'], 1.2, delta=0.02)

    def test_scale_immune_to_gyro_bias(self):
        """尺度比必须去均值回归（评审 @KKK555-MAX 指出）。

        IMU 常值零偏让「截距为 0」的最小二乘产生偏置：零偏 0.05 rad/s 时
        旧实现偏到 0.984（1.6%），而文档把 scale 当独立检查项用。互相关本来就是
        去均值的，回归同口径后尺度比应与零偏无关。
        """
        scales = []
        for bias in (0.0, 0.005, 0.05, 0.2):
            _, res = core.trial(seed=0, offset=0.050, imu_bias=bias)
            scales.append(res['scale'])
        for s in scales:
            self.assertAlmostEqual(s, 1.0, delta=0.005)
        # 零偏从 0 拉到 0.2 rad/s，尺度比漂移应在千分之一量级
        self.assertLess(max(scales) - min(scales), 0.001)

    def test_frame_start_timestamp_bias(self):
        """时间戳打在帧开始时，估计值会少半帧（30 Hz 时约 16.7 ms）。"""
        _, res = core.trial(seed=0, offset=0.050, cam_rate=30.0,
                            timestamp_at='start')
        self.assertAlmostEqual(res['offset'], 0.050 - 1.0 / 60.0, delta=0.003)


class TestTrimBuffers(unittest.TestCase):
    """缓冲区按消息时间戳裁窗，不依赖节点时钟（评审 @KKK555-MAX 指出）。"""

    @staticmethod
    def _buf(t0: float, dt: float, n: int):
        from collections import deque
        return deque((t0 + dt * k, (0.0, 0.0, 0.0)) for k in range(n))

    def test_window_counts_from_message_stamp(self):
        # 消息时间戳基准设在 1000 s（远早于节点时钟）：若按节点时钟裁，
        # 整个缓冲区会被一次裁空，这里应保留最近 8 s
        imu = self._buf(1000.0, 0.01, 1001)      # 100 Hz，10 s
        cam = self._buf(1000.0, 0.1, 101)        # 10 Hz，10 s
        core.trim_buffers((imu, cam), 8.0)
        self.assertAlmostEqual(imu[0][0], 1002.0, delta=0.011)
        self.assertAlmostEqual(cam[0][0], 1002.0, delta=0.101)
        self.assertEqual(len(imu), 801)
        self.assertEqual(len(cam), 81)

    def test_both_windows_aligned_to_newest_stamp(self):
        # 相机流滞后 5 s（断流或时间基准不同）：以最新消息时刻为准，
        # 两路窗口落在同一时间跨度上
        imu = self._buf(1000.0, 0.01, 1001)      # 到 1010 s
        cam = self._buf(1000.0, 0.1, 51)         # 到 1005 s
        core.trim_buffers((imu, cam), 8.0)
        self.assertAlmostEqual(cam[0][0], 1002.0, delta=0.101)
        self.assertLessEqual(cam[-1][0], 1010.0)

    def test_empty_buffers_are_safe(self):
        from collections import deque
        core.trim_buffers((deque(), deque()), 8.0)      # 不应抛异常
        core.trim_buffers((), 8.0)


class TestExposureVsFramePeriod(unittest.TestCase):
    """曝光时长与帧周期是两个独立参数（评审 @1huangna 指出）。

    ``timestamp_at='start'`` 时等效测量时刻是 ``t_stamp + exposure/2``，因此
    偏差是 ``-exposure/2`` 而不是固定的半帧；``timestamp_at='mid'`` 时窗口关于
    时间戳对称，偏差与曝光长短无关。
    """

    def test_reports_exposure_and_frame_period(self):
        sim = core.simulate(seed=0, cam_rate=30.0)
        self.assertAlmostEqual(sim['frame_period'], 1.0 / 30.0, places=12)
        # 默认 exposure = 帧周期，保持旧行为（整帧窗口平均）
        self.assertAlmostEqual(sim['exposure'], 1.0 / 30.0, places=12)
        sim2 = core.simulate(seed=0, cam_rate=30.0, exposure=0.008)
        self.assertAlmostEqual(sim2['exposure'], 0.008, places=12)

    def test_rejects_exposure_out_of_range(self):
        with self.assertRaises(ValueError):
            core.simulate(cam_rate=30.0, exposure=0.05)   # 长于帧周期
        with self.assertRaises(ValueError):
            core.simulate(cam_rate=30.0, exposure=0.0)

    def test_frame_start_bias_follows_exposure_not_frame_period(self):
        """曝光取 1/4 帧时，偏差应是 -Texp/2（约 -4.17 ms），不是 -16.67 ms。"""
        exposure = 0.25 / 30.0
        _, res = core.trial(seed=0, offset=0.050, cam_rate=30.0,
                            timestamp_at='start', exposure=exposure)
        self.assertAlmostEqual(res['offset'], 0.050 - exposure / 2.0, delta=0.002)

    def test_frame_start_bias_scales_with_exposure(self):
        """偏差随曝光时长线性增长，Texp = T 时才退化为半帧。"""
        biases = []
        for ratio in (0.25, 0.5, 1.0):
            exposure = ratio / 30.0
            _, res = core.trial(seed=0, offset=0.050, cam_rate=30.0,
                                timestamp_at='start', exposure=exposure)
            biases.append((res['offset'] - 0.050) * 1000.0)
        for bias, ratio in zip(biases, (0.25, 0.5, 1.0)):
            self.assertAlmostEqual(bias, -ratio * 1000.0 / 60.0, delta=0.6)

    def test_mid_timestamp_immune_to_short_exposure(self):
        """时间戳在曝光中点时，曝光变短不引入系统偏差。"""
        for exposure in (0.2 / 30.0, 0.5 / 30.0, 1.0 / 30.0):
            _, res = core.trial(seed=0, offset=0.050, cam_rate=30.0,
                                exposure=exposure)
            self.assertAlmostEqual(res['offset'], 0.050, delta=0.002,
                                   msg=f'exposure={exposure:.4f} s')


class TestAttitude(unittest.TestCase):
    def test_constant_rate_integration(self):
        """绕单轴恒定角速度积分 1 s，角度应为 omega * 1 s。"""
        t = np.linspace(0.0, 1.0, 201)
        w = np.tile([0.0, 0.0, 0.5], (len(t), 1))
        q = core.integrate_attitude(t, w)
        angle = np.degrees(2.0 * np.arccos(min(1.0, abs(q[-1, 0]))))
        self.assertAlmostEqual(angle, np.degrees(0.5), delta=0.5)

    def test_compensation_reduces_error(self):
        """补偿 50 ms 偏移后，姿态 RMS 误差应显著下降。"""
        sim, res = core.trial(seed=0, offset=0.050)
        before = core.attitude_error(0.0, sim)
        after = core.attitude_error(res['offset'], sim)
        self.assertLess(after['rms_deg'], 0.5 * before['rms_deg'])

    def test_error_is_zero_for_small_offset(self):
        """偏移为 0 时，未补偿与"补偿 0"应当是同一件事。"""
        sim, _ = core.trial(seed=0, offset=0.0)
        a = core.attitude_error(0.0, sim)
        b = core.attitude_error(0.0, sim)
        self.assertAlmostEqual(a['max_deg'], b['max_deg'], places=6)
        self.assertLess(a['max_deg'], 1.5)

    def test_truth_attitude_follows_excitation(self):
        """弱激励下，误差统计用的真值必须与仿真轨迹一致（含 excitation 缩放）。

        回归测试：原实现用 omega(t) 重算真值，激励 0.5 且完全补偿时会把
        0.014° 的离散化残差报成 18.7°，因为真值幅值是仿真用的两倍。
        """
        for excitation in (0.5, 0.1):
            sim = core.simulate(seed=0, excitation=excitation,
                                cam_noise=0.0, imu_noise=0.0, offset=0.050)
            err = core.attitude_error(0.050, sim)
            self.assertLess(err['rms_deg'], 0.1,
                            msg=f'excitation={excitation} 时残余误差应趋于 0')


if __name__ == '__main__':
    unittest.main()
