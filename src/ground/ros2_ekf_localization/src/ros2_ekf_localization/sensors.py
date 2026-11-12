"""传感器噪声模型：为离线仿真合成带误差的里程计、IMU 与位置观测。

三类误差分开建模，因为它们对定位结果的影响完全不同：

- **比例误差**（轮径标定不准、左右轮不等）：随行驶距离**累积**，是航位推算
  漂移的主因，靠滤波无法消除，只能靠绝对观测纠正；
- **高斯白噪声**：不累积，滤波器可以平滑掉；
- **常值偏置**（IMU 零偏）：介于两者之间，长时间会让航向缓慢偏离。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .ekf import normalize_angle


@dataclass
class OdometryNoise:
    """轮式里程计噪声：比例误差 + 白噪声。"""

    scale_v: float = 0.02    # 线速度比例误差（轮径标定误差）
    scale_w: float = 0.05    # 角速度比例误差（左右轮径不等 / 打滑）
    sigma_v: float = 0.03    # 线速度白噪声 (m/s)
    sigma_w: float = 0.02    # 角速度白噪声 (rad/s)

    def corrupt(self, v: float, w: float, rng: np.random.Generator) -> tuple[float, float]:
        """把真值 (v, w) 变成里程计读到的 (v, w)。"""
        v_m = v * (1.0 + self.scale_v) + rng.normal(0.0, self.sigma_v)
        w_m = w * (1.0 + self.scale_w) + rng.normal(0.0, self.sigma_w)
        return float(v_m), float(w_m)


@dataclass
class ImuNoise:
    """IMU 航向观测噪声：常值偏置 + 白噪声。"""

    sigma_yaw: float = 0.03   # 航向白噪声 (rad)
    bias_yaw: float = 0.01    # 航向常值偏置 (rad)

    def observe(self, theta: float, rng: np.random.Generator) -> float:
        return float(normalize_angle(theta + self.bias_yaw + rng.normal(0.0, self.sigma_yaw)))


class PositionSensor:
    """低频绝对位置观测（GPS / UWB / 视觉路标）。

    rate_hz <= 0 表示不提供位置观测，用于对照实验。
    """

    def __init__(self, sigma: float = 0.15, rate_hz: float = 1.0) -> None:
        self.sigma = float(sigma)
        self.rate_hz = float(rate_hz)
        self._next_t = 0.0

    def available(self, t: float) -> bool:
        """该时刻是否产生一次位置观测（按固定频率抽样）。"""
        if self.rate_hz <= 0.0:
            return False
        if t + 1e-12 >= self._next_t:
            self._next_t += 1.0 / self.rate_hz
            return True
        return False

    def observe(self, x: float, y: float, rng: np.random.Generator) -> tuple[float, float]:
        return float(x + rng.normal(0.0, self.sigma)), float(y + rng.normal(0.0, self.sigma))


def integrate_odometry(
    v_seq: np.ndarray, w_seq: np.ndarray, dt: float, x0: np.ndarray | None = None
) -> np.ndarray:
    """纯航位推算：直接积分里程计输入，作为对照基线。

    返回 (N, 3) 的位姿序列。
    """
    x = np.zeros(3, dtype=float) if x0 is None else np.asarray(x0, dtype=float).copy()
    out = np.zeros((len(v_seq), 3), dtype=float)
    for k, (v, w) in enumerate(zip(v_seq, w_seq)):
        out[k] = x
        x = np.array(
            [
                x[0] + v * math.cos(x[2]) * dt,
                x[1] + v * math.sin(x[2]) * dt,
                normalize_angle(x[2] + w * dt),
            ]
        )
    return out
