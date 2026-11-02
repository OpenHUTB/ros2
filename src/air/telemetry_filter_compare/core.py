#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
telemetry_filter_compare/core.py - 无人机遥测高度数据滤波对比核心

对含高斯噪声的高度遥测数据，对比两种滤波器：
    1. 滑动平均 (Moving Average, 窗口 N)
    2. 一维卡尔曼滤波 (标量恒增益模型: 状态=高度, 无控制输入)
评价指标: 均方根误差 RMSE (相对真实高度)。
"""

import math
import random

_NOISE_RNG = random.Random(2026)


def random_noise(std: float = 2.0) -> float:
    """节点用高斯噪声 (固定种子保证可复现)。"""
    return _NOISE_RNG.gauss(0.0, std)


def moving_average(data, window=5):
    """滑动平均, 前沿用现有数据填充 (窗口从 1 逐渐增大)。"""
    out = []
    for i in range(len(data)):
        lo = max(0, i - window + 1)
        seg = data[lo:i + 1]
        out.append(sum(seg) / len(seg))
    return out


class ScalarKalman:
    """标量卡尔曼滤波 (状态 x = 高度, 观测 z = 含噪高度)。"""

    def __init__(self, process_var=0.1, measurement_var=4.0, x0=0.0, p0=100.0):
        self.q = process_var   # 过程噪声方差
        self.r = measurement_var  # 观测噪声方差
        self.x = x0
        self.p = p0

    def update(self, z: float, dt: float = 1.0) -> float:
        # 预测 (状态恒定模型)
        self.p += self.q * dt
        # 更新
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1.0 - k)
        return self.x


def generate_noisy_altitude(true_alt_fn, n=300, noise_std=2.0, seed=42):
    """生成含噪高度序列。true_alt_fn(t) 返回真实高度。"""
    rng = random.Random(seed)
    return [true_alt_fn(t) + rng.gauss(0.0, noise_std) for t in range(n)]


def rmse(filtered, truth):
    return math.sqrt(sum((f - t) ** 2 for f, t in zip(filtered, truth)) / len(truth))


def compare_filters(noisy, truth, window=9):
    """返回 {'raw': .., 'moving_average': .., 'kalman': ..} 的 RMSE 对比。"""
    ma = moving_average(noisy, window)
    kf = ScalarKalman(process_var=0.05, measurement_var=noise_variance(noisy, truth))
    kalman_out = [kf.update(z) for z in noisy]
    return {
        'raw': rmse(noisy, truth),
        'moving_average': rmse(ma, truth),
        'kalman': rmse(kalman_out, truth),
    }


def noise_variance(noisy, truth):
    """用残差估计观测噪声方差 (卡尔曼 R 参数)。"""
    n = len(truth)
    mean_sq = sum((z - t) ** 2 for z, t in zip(noisy, truth)) / n
    return max(1e-6, mean_sq)
