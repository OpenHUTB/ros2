#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""telemetry_filter_compare 单元测试（不依赖 ROS）。"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import (  # noqa: E402
    ScalarKalman, compare_filters, generate_noisy_altitude, moving_average,
    rmse,
)


def true_alt(t):
    return 10.0 + 0.05 * t  # 缓慢爬升


def test_moving_average_length():
    data = [1.0, 2.0, 3.0, 4.0, 5.0]
    assert len(moving_average(data, 3)) == 5


def test_moving_average_smooths_spike():
    data = [0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0]
    ma = moving_average(data, 3)
    # 尖峰位置的滤波值应被显著压低
    assert ma[3] < 50.0


def test_kalman_converges():
    kf = ScalarKalman(process_var=0.01, measurement_var=4.0, x0=0.0, p0=100.0)
    for _ in range(200):
        kf.update(10.0 + math.sin(_ * 0.1))
    # 滤波值应接近观测均值附近 (10 上下), 且协方差收敛变小
    assert 5.0 < kf.x < 15.0
    assert kf.p < 1.0


def test_kalman_better_than_raw():
    noisy = generate_noisy_altitude(true_alt, n=300, noise_std=2.0)
    truth = [true_alt(t) for t in range(300)]
    res = compare_filters(noisy, truth)
    assert res['kalman'] < res['raw']
    assert res['moving_average'] < res['raw']


def test_kalman_beats_lagging_ma_on_ramp():
    """斜坡信号下卡尔曼应优于滞后明显的滑动平均。"""
    noisy = generate_noisy_altitude(true_alt, n=400, noise_std=2.0, seed=7)
    truth = [true_alt(t) for t in range(400)]
    res = compare_filters(noisy, truth, window=15)
    assert res['kalman'] <= res['moving_average'] * 1.2


def test_rmse_zero_for_equal():
    assert rmse([1.0, 2.0], [1.0, 2.0]) == 0.0
