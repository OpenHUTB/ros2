#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""未建模动力学：气动阻力、风场、挂载载荷、地面效应。

力统一在**机体系**里定义再旋到世界系：各向异性阻力、载荷偏置都随姿态一起转，
同一组系数对任意姿态成立，残差网络学到的映射也与姿态解耦。
本模块只依赖 numpy，ROS 侧（Python 3.8）可直接导入。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

GRAVITY = 9.81


# --------------------------------------------------------------------------- #
# 气动阻力
# --------------------------------------------------------------------------- #
@dataclass
class DragModel:
    """机体系各向异性二次阻力。

    水平/垂直分开系数：四旋翼是扁的，竖直方向迎风面积远大于水平方向。
    系数按 10 m/s 时阻力约占重力 65% 标定（对应约 33° 前倾），与实物量级一致；
    若按"每 m/s² 几牛"随手取，10 m/s 下阻力会超过推力上限，轨迹直接发散。
    """

    k_h: float = 0.055      # 水平二次项 N/(m/s)²
    k_v: float = 0.090      # 垂直二次项 N/(m/s)²
    linear: float = 0.020   # 线性项 N/(m/s)，低速时主导

    def body_force(self, v_b: np.ndarray) -> np.ndarray:
        """相对气流的机体系速度 → 机体系阻力（不含风，风由调用方先扣除）。"""
        v = np.asarray(v_b, dtype=float)
        s_h = math.hypot(v[0], v[1])
        s_v = abs(v[2])
        f = np.array([self.k_h * s_h * v[0],
                      self.k_h * s_h * v[1],
                      self.k_v * s_v * v[2]])
        return -(f + self.linear * v)


# --------------------------------------------------------------------------- #
# 风场
# --------------------------------------------------------------------------- #
@dataclass
class WindModel:
    """世界系风场：常值平均风 + 一阶 Gauss-Markov（OU 过程）阵风。

    OU 是 Dryden 谱的时域等价形式，离散化 a=exp(-dt/τ)、
    s=σ√(1−a²) 是精确解（不是欧拉近似），任意 dt 下方差都严格等于 σ²。
    """

    mean: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gust: float = 0.0
    tau: float = 1.5
    vertical_ratio: float = 0.5      # 垂直阵风通常远小于水平
    seed: int = 0

    def __post_init__(self) -> None:
        self.mean = np.asarray(self.mean, dtype=float)
        self.reset()

    def reset(self) -> None:
        self._rng = np.random.default_rng(self.seed)
        self._fluct = np.zeros(3)

    def update(self, dt: float) -> None:
        """推进一个物理子步。"""
        if self.gust <= 0.0:
            return
        a = math.exp(-dt / max(self.tau, 1e-6))
        s = self.gust * math.sqrt(max(1.0 - a * a, 0.0))
        w = self._rng.standard_normal(3)
        w[2] *= self.vertical_ratio
        self._fluct = a * self._fluct + s * w

    @property
    def vel(self) -> np.ndarray:
        """当前世界系风速。"""
        return self.mean + self._fluct

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.vel))


# --------------------------------------------------------------------------- #
# 载荷
# --------------------------------------------------------------------------- #
@dataclass
class PayloadModel:
    """挂在机体下方的集中质量。"""

    mass: float = 0.0
    offset: Tuple[float, float, float] = (0.0, 0.0, -0.075)

    @property
    def ratio(self) -> float:
        """相对 0.95 kg 空机质量的比值。"""
        return self.mass / 0.95


# --------------------------------------------------------------------------- #
# 组合
# --------------------------------------------------------------------------- #
@dataclass
class PlantConfig:
    """一整套未建模效应的参数。"""

    drag: DragModel = field(default_factory=DragModel)
    wind: WindModel = field(default_factory=WindModel)
    payload: PayloadModel = field(default_factory=PayloadModel)
    # 地面效应：近地时同一转速产生更大推力。起降段与低空悬停都会碰到，
    # 依赖高度而非速度，是检验网络是否真的学到状态依赖的关键项。
    ground_effect: float = 0.12
    ground_effect_h: float = 0.25

    def ground_scale(self, z: float) -> float:
        if self.ground_effect <= 0.0:
            return 0.0
        return self.ground_effect * math.exp(-max(z, 0.0) / self.ground_effect_h)

    def drag_force(self, v_world: np.ndarray, R: np.ndarray) -> np.ndarray:
        """世界系速度 + 姿态 → 世界系阻力（已扣除风）。"""
        v_b = R.T @ (np.asarray(v_world, float) - self.wind.vel)
        return R @ self.drag.body_force(v_b)

    def reset(self) -> None:
        self.wind.reset()

    def describe(self) -> Dict[str, object]:
        return {
            "payload_kg": round(self.payload.mass, 4),
            "wind_mean": [round(float(x), 2) for x in self.wind.mean],
            "wind_gust": round(self.wind.gust, 2),
            "wind_speed": round(self.wind.speed, 2),
        }


# --------------------------------------------------------------------------- #
# 预设档位
# --------------------------------------------------------------------------- #
PAYLOAD_LEVELS: Dict[str, float] = {
    "none": 0.0,        # 空载
    "light": 0.25,      # 轻载 +250 g
    "heavy": 0.50,      # 重载 +500 g
    "extra": 0.75,      # 超载 +750 g（训练集之外，测外推与在线自适应）
}

WIND_LEVELS: Dict[str, Dict[str, object]] = {
    "calm": dict(mean=(0.0, 0.0, 0.0), gust=0.0),
    "breeze": dict(mean=(1.5, 0.4, 0.0), gust=0.9),
    "windy": dict(mean=(3.2, -1.1, 0.0), gust=1.8),
}

#: 训练集覆盖的档位（extra 与未见过的风种子留作测试）
TRAIN_PAYLOADS = ("none", "light", "heavy")
TRAIN_WINDS = ("calm", "breeze", "windy")


def make_plant(payload: str = "heavy", wind: str = "windy",
               seed: int = 0, drag: Optional[DragModel] = None,
               ground_effect: float = 0.12) -> PlantConfig:
    """按档位名构造 PlantConfig。

    ``seed`` 只影响风的随机实现；同一 ``(payload, wind, seed)`` 完全可复现。
    """
    if payload not in PAYLOAD_LEVELS:
        raise KeyError("未知载荷档位 %r，可选 %s" % (payload, list(PAYLOAD_LEVELS)))
    if wind not in WIND_LEVELS:
        raise KeyError("未知风档位 %r，可选 %s" % (wind, list(WIND_LEVELS)))
    kw = dict(WIND_LEVELS[wind])
    w = WindModel(gust=float(kw.pop("gust")), seed=seed, **kw)  # type: ignore[arg-type]
    return PlantConfig(
        drag=drag if drag is not None else DragModel(),
        wind=w,
        payload=PayloadModel(mass=PAYLOAD_LEVELS[payload]),
        ground_effect=ground_effect,
    )
