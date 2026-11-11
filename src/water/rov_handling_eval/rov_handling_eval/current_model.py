# -*- coding: utf-8 -*-
"""三层级联洋流模型（纯 numpy，可离线测试）。

    v_c(t, z, d) = v_gm(t) + v_shear(z) + v_turb(d)

1. Gauss-Markov 时间波动：一阶自回归，`v_{k+1} = a·v_k + σ·ξ`，a = exp(−dt/τ)；
2. 深度剪切：`v(z) = v0 · ((z_ref − z)/z_ref)^α`，越深越慢；
3. 空间湍流：`δv = σ_t · exp(−d/L) · ξ`，随距离衰减。

设计要点：三层都归一到「平均流速」这一个参数上，
这样"洋流 0.3 m/s"在每次实验、每个种子里含义一致，实验才可比较。
"""

import math
import random
from dataclasses import dataclass

import numpy as np


@dataclass
class CurrentParams:
    mean_speed: float = 0.0      # 目标平均流速 m/s
    direction_deg: float = 90.0  # 流向，0=+x，90=+y（世界系，水平面）
    tau: float = 8.0             # Gauss-Markov 相关时间 s
    sigma_ratio: float = 0.25    # 波动标准差 / 平均流速
    shear_alpha: float = 0.6     # 深度剖面指数
    z_ref: float = 10.0          # 参考深度（剪切零点）m
    turb_sigma: float = 0.05     # 湍流强度（相对平均流速）
    turb_length: float = 2.0     # 湍流相关长度 m
    seed: int = 0


class CurrentField:
    """可复现的洋流场。同一 seed + 同一时间步序列 → 同一结果。"""

    def __init__(self, params: CurrentParams):
        self.p = params
        self.rng = random.Random(params.seed)
        self._gm = np.zeros(2)          # 水平两维的 Gauss-Markov 状态
        self._last_v = self._mean_vector()

    def _mean_vector(self) -> np.ndarray:
        th = math.radians(self.p.direction_deg)
        return np.array([math.cos(th), math.sin(th)]) * self.p.mean_speed

    def reset(self):
        self.rng = random.Random(self.p.seed)
        self._gm = np.zeros(2)

    def step(self, dt: float, z: float, distance: float = 0.0) -> np.ndarray:
        """推进一个时间步，返回世界系流速 [vx, vy, vz]（m/s）。"""
        p = self.p
        mean = self._mean_vector()

        # 1) Gauss-Markov：稳态方差 = σ²，用 a = exp(-dt/τ) 保证与 dt 无关
        a = math.exp(-dt / max(p.tau, 1e-6))
        sigma = p.sigma_ratio * p.mean_speed
        noise_scale = sigma * math.sqrt(max(1.0 - a * a, 0.0))
        for i in range(2):
            self._gm[i] = a * self._gm[i] + noise_scale * self.rng.gauss(0.0, 1.0)

        # 2) 深度剪切：z 向上，越深（z 越小）流速越接近 0，表层最快
        depth = max(-z, 0.0)
        ratio = min(depth / max(p.z_ref, 1e-6), 1.0)
        shear = (1.0 - ratio) ** p.shear_alpha if p.shear_alpha > 0 else 1.0

        # 3) 空间湍流：与 ROV 距离相关，指数衰减
        turb = p.turb_sigma * p.mean_speed * math.exp(-max(distance, 0.0) / max(p.turb_length, 1e-6))

        v = (mean + self._gm) * shear + turb * np.array([self.rng.gauss(0, 1), self.rng.gauss(0, 1)])
        return np.array([v[0], v[1], 0.0])

    def magnitude(self, z: float, distance: float = 0.0) -> float:
        """当前流速大小（不推进状态，用于评测节点显示）。"""
        v = np.asarray(self._last_v, dtype=float)
        return float(np.linalg.norm(v[:2]))
