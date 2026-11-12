# -*- coding: utf-8 -*-
"""运动模型：把一帧里程计增量"打散"成一群粒子的增量。

用的是概率机器人学里最常用的**里程计增量模型**：

    对每个粒子独立采样
        ds_hat  = ds · (1 + n₁)           n₁ ~ N(0, a₁·|ds| + a₂·|dθ|)
        dθ_hat  = dθ · (1 + n₂) + n₃      n₂ ~ N(0, a₃·|dθ| + a₄·|ds|)
    再沿 dθ_hat 做中点积分推进位姿

四个噪声参数 α₁~α₄ 的物理含义：
    α₁  直行时"走多远"的不确定性（轮径误差、纵向滑移）
    α₂  直行时"转多少"的不确定性（左右轮不等）
    α₃  转向时"转多少"的不确定性（轮距误差、横向刮擦）
    α₄  转向时"走多远"的不确定性（打滑造成的额外位移）

参数靠手调很难调好：调大 → 定位抖动；调小 → 真实打滑时跟不上、直接发散。
本模块另外提供一个可选的**神经网络修正项**（`nn_motion.NNMotionModel`），
用来预测"里程计增量 → 真实增量"的偏差与噪声标度，见文档 §5。
"""

import math

import numpy as np

from .geometry import wrap_angle


class MotionModel(object):
    """里程计增量模型。

    参数
    ----
    alpha : (a1, a2, a3, a4)
    min_noise : float
        噪声下限（米/弧度）。避免在完全静止时所有粒子重合退化成单点。
    """

    def __init__(self, alpha=(0.10, 0.02, 0.10, 0.02), min_noise=1e-4):
        self.alpha = tuple(float(a) for a in alpha)
        self.min_noise = float(min_noise)

    def sample(self, particles, delta, rng):
        """把 delta=(ds, dθ)（车体系）施加到每个粒子上。

        particles : (N, 3) 原地修改的副本，返回新数组
        delta     : (ds, dθ)
        """
        particles = np.array(particles, dtype=np.float64, copy=True)
        ds, dth = float(delta[0]), float(delta[1])
        n = particles.shape[0]
        a1, a2, a3, a4 = self.alpha

        # 噪声标准差随运动量线性增长（这是该模型的核心假设）
        std_s = math.sqrt(a1 * ds * ds + a2 * dth * dth) + self.min_noise
        std_t = math.sqrt(a3 * dth * dth + a4 * ds * ds) + self.min_noise

        ds_hat = ds * (1.0 + rng.normal(0.0, 1.0, n) * (std_s / max(abs(ds), 1e-6) if abs(ds) > 1e-6 else 1.0))
        dth_hat = dth + rng.normal(0.0, 1.0, n) * std_t

        # 中点积分推进
        theta = particles[:, 2]
        mid = theta + 0.5 * dth_hat
        particles[:, 0] += ds_hat * np.cos(mid)
        particles[:, 1] += ds_hat * np.sin(mid)
        particles[:, 2] = np.array([wrap_angle(v) for v in theta + dth_hat])
        return particles

    def sample_with_correction(self, particles, delta, rng, corrector=None, context=None):
        """带神经网络修正的采样。

        corrector(context) 返回 (ds_fixed, dth_fixed, noise_scale)；
        context 通常是一小段里程计统计量。noise_scale 会等比放大 α，
        使"打滑时粒子自动散开、正常时自动收紧"。
        """
        if corrector is None:
            return self.sample(particles, delta, rng)

        ds_fixed, dth_fixed, noise_scale = corrector(context)
        scale = float(np.clip(noise_scale, 0.25, 4.0))
        saved = self.alpha
        try:
            self.alpha = tuple(a * scale for a in saved)
            return self.sample(particles, (ds_fixed, dth_fixed), rng)
        finally:
            self.alpha = saved


def odometry_delta(prev_pose, cur_pose):
    """两帧里程计位姿 → (ds, dθ)。

    与 `geometry.relative_pose` 的区别：这里**只用航向差**，
    位移长度取两点距离，符合"编码器给出走了多远、转了多少"的物理事实。
    """
    dx = cur_pose[0] - prev_pose[0]
    dy = cur_pose[1] - prev_pose[1]
    ds = math.hypot(dx, dy)
    dth = wrap_angle(cur_pose[2] - prev_pose[2])
    return (ds, dth)
