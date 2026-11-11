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

    def test_frame_start_timestamp_bias(self):
        """时间戳打在帧开始时，估计值会少半帧（30 Hz 时约 16.7 ms）。"""
        _, res = core.trial(seed=0, offset=0.050, cam_rate=30.0,
                            timestamp_at='start')
        self.assertAlmostEqual(res['offset'], 0.050 - 1.0 / 60.0, delta=0.003)


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
