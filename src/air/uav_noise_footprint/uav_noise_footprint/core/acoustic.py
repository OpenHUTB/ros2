#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""声学模型：桨盘功率 → 声功率级 → 任意距离的声压级。

传播模型只包含两项物理效应：

1. **球面扩散**：点声源的声压级随距离按 −20·log10(r) 衰减（自由场）；
2. **大气吸收**：α ≈ 0.5 dB/100 m（@500 Hz，20 °C、50 % RH 的典型值，
   见 ISO 9613-1 表 2 量级），随传播距离线性累积。

于是::

    Lp(r) = Lw − 20·log10(r) − α·r            （r 单位 m，α 单位 dB/m）

经验公式 ``Lw = 10·log10(P) + K`` 把桨盘功率 P（W）映射为 A 计权声功率级
Lw（dB）。它来自"声功率与输入功率近似成正比"的标度关系：若声效率 η 恒定，
则 W_acoustic = η·P，Lw = 10·log10(W_acoustic/W0) = 10·log10(P) + 10·log10(η·1e-12)。
小型多旋翼桨盘声效率典型在 10^-5 量级（sUAS 实测声功率 90–100 dB 量级，
参见 Deppe et al. 2020 对小型无人机噪声的实测综述与 Alexander et al. 的
旋翼噪声标度实验——此处只取其"幂律 + 常数"的近似形式）。``K`` 是标定
常数：取基准机型桨盘功率 P=300 W，要求 100 m 处约 55 dB，即
Lw ≈ 55 + 40 + 0.5 = 95.5 dB → K = 95.5 − 10·log10(300) ≈ 70.7。
这是工程近似，不是严格气动声学预测（后者需要桨叶谐波 BPM 模型）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

#: 大气吸收系数（dB/m）：0.5 dB/100 m @ 500 Hz
ALPHA_DB_PER_M = 0.005
#: 最小统计距离（m）：r 小于它时按它计，避免 log 奇点
MIN_DISTANCE = 1.0


@dataclass(frozen=True)
class AcousticModel:
    """一套标定好的声源 + 传播参数。"""

    rotor_power: float = 300.0   # 桨盘功率（W）
    k_const: float = 70.73       # Lw = 10·log10(P) + K 的标定常数
    alpha: float = ALPHA_DB_PER_M

    # ---------------------------------------------------------------- 声源
    def sound_power_level(self, power: float | None = None) -> float:
        """桨盘功率（W）→ A 计权声功率级 Lw（dB）。"""
        p = float(self.rotor_power if power is None else power)
        if p <= 0.0:
            raise ValueError("桨盘功率必须为正: %r" % (p,))
        return 10.0 * math.log10(p) + self.k_const

    # ---------------------------------------------------------------- 传播
    def lp_at(self, distance: float, power: float | None = None) -> float:
        """距离声源 ``distance``（m）处的声压级 Lp（dB）。

        Lp(r) = Lw − 20·log10(r) − α·r，r < 1 m 按 1 m 截断。
        """
        r = max(float(distance), MIN_DISTANCE)
        return self.sound_power_level(power) - 20.0 * math.log10(r) - self.alpha * r

    def lp_from(self, source, receiver) -> float:
        """三维两点 (x, y, z) 之间的声压级。"""
        return self.lp_at(math.dist(source, receiver))

    # ---------------------------------------------------------------- 校准
    def calibration_check(self, r: float = 100.0) -> float:
        """返回基准机型在 ``r`` 米处的 Lp，用于确认标定（应约 55 dB）。"""
        return self.lp_at(r)


#: 全局默认模型（模块级共享，测试与节点都用它）
DEFAULT = AcousticModel()


def energy_sum(levels_db) -> float:
    """能量叠加：若干个 dB 值合成一个总 dB 值 10·log10(Σ10^(L/10))。"""
    total = 0.0
    for lv in levels_db:
        total += 10.0 ** (float(lv) / 10.0)
    return 10.0 * math.log10(total) if total > 0.0 else -math.inf
