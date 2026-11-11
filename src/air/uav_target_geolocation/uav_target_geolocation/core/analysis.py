#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""精度评估：蒙特卡洛打点 + 从实测位姿算误差。

两种输入走同一套统计口径：
- 解析模式（``monte_carlo``）：只按几何 + 噪声打点，不跑仿真，用来扫
  交会角这类"参数 vs 精度"的曲线；
- 实测模式（``from_station_directions``）：喂 MuJoCo 闭环里真实采到的
  位置与视线，算的是同一组指标，两种模式的数可以直接对照。

误差源分两类，这一点决定了"多站交会到底能不能救回来"：
- **逐帧抖动**（``sigma_deg``）：云台稳定环的残余抖动，按 1/√N 平均掉；
- **单站零位偏置**（``bias_deg``）：每个站的视轴标定误差，一整个架次
  都不变，平均不掉——多站交会之所以有用，正是因为它把"各自的偏置"
  变成两个不同方向的约束。
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

import numpy as np

from .cases import GeoCase
from .fusion import (angular_spread_deg, fused_from_directions, mean_direction,
                     summarize)
from .geometry import (Ray, horizontal_intersection_angle_deg, los_to,
                       pairwise_min_angle_deg, unit)

#: 悬停时四旋翼的姿态抖动量（1σ，度）
TILT_SIGMA_DEG = 1.2
#: 四旋翼总质量（kg），与 uav_mujoco_sim/models/quadrotor.xml 一致
QUAD_MASS_KG = 0.95
#: 重力加速度
G = 9.81


def wind_tilt_deg(wind_n: Sequence[float]) -> float:
    """悬停被水平外力吹着时，机体的倾角（度）。

    推力要抵消重力与外力的合力，倾角满足 tanθ = F_h / (mg)。这一项在
    "拿指令姿态当真实姿态"的对照里不会被平均掉——它是一整个架次的固定偏差。
    """
    fh = float(math.hypot(float(wind_n[0]), float(wind_n[1])))
    if fh <= 0.0:
        return 0.0
    return math.degrees(math.atan2(fh, QUAD_MASS_KG * G))


def _rotate_small(los: np.ndarray, ax_deg: float, ay_deg: float) -> np.ndarray:
    """把视线绕机体 x / y 轴各转一个小角度（度）。"""
    a = math.radians(ax_deg)
    b = math.radians(ay_deg)
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, math.cos(a), -math.sin(a)],
                   [0.0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0.0, math.sin(b)],
                   [0.0, 1.0, 0.0],
                   [-math.sin(b), 0.0, math.cos(b)]])
    return unit(Rx @ Ry @ los, "los")


def single_station_error(origin: Sequence[float], direction: Sequence[float],
                         truth: Sequence[float],
                         ground_z: float = 0.0) -> Optional[float]:
    """单条视线的落点误差；打不到地面返回 None。"""
    try:
        p = Ray(origin=origin, direction=direction).ground_hit(ground_z)
    except ValueError:
        return None
    t = np.asarray(truth, dtype=float).reshape(-1)
    return float(math.hypot(p[0] - t[0], p[1] - t[1]))


def from_station_directions(stations: Sequence[Sequence[float]],
                            directions: Sequence[np.ndarray],
                            truth: Sequence[float],
                            ground_z: float = 0.0) -> Dict[str, object]:
    """给定每站的平均视线，算单站误差与交会误差。"""
    if len(stations) != len(directions):
        raise ValueError("站数与视线数不一致")
    per_station = [single_station_error(p, d, truth, ground_z)
                   for p, d in zip(stations, directions)]
    rays = [Ray(origin=p, direction=d) for p, d in zip(stations, directions)]
    fused = fused_from_directions(stations, directions, ground_z=ground_z)
    t = np.asarray(truth, dtype=float).reshape(-1)
    hit = None if not fused.ok else fused.error_to(np.array([t[0], t[1], 0.0]))
    return {
        "per_station_m": [None if e is None else round(float(e), 4)
                          for e in per_station],
        "fused_m": None if hit is None else round(float(hit), 4),
        "fused_ok": bool(fused.ok),
        "fused_reason": fused.reason,
        "intersection_deg": (float(horizontal_intersection_angle_deg(rays))
                             if len(rays) > 1 else float("nan")),
        "condition": float(fused.condition),
        "residual_m": float(fused.residual_m),
    }


def monte_carlo(case: GeoCase, n: int = 600, seed: int = 0,
                samples_per_station: Optional[int] = None,
                tilt_sigma_deg: float = TILT_SIGMA_DEG,
                tilt_bias: Optional[float] = None) -> Dict[str, object]:
    """只按几何打点：逐帧抖动会被平均，单站偏置不会。"""
    if n < 10:
        raise ValueError(f"蒙特卡洛样本太少：{n}")
    k = int(case.samples if samples_per_station is None else samples_per_station)
    if k < 1:
        raise ValueError("每站采样帧数至少为 1")
    rng = np.random.default_rng(seed)
    truth = np.asarray(case.target, dtype=float)
    stations = [s.position for s in case.stations]
    sigma = float(case.sigma_deg)
    bias = float(case.bias_deg)

    tilt_bias_deg = (wind_tilt_deg(case.wind_n) if tilt_bias is None
                     else float(tilt_bias))

    single_err: List[float] = []
    level_err: List[float] = []
    fused_err: List[float] = []
    spreads: List[float] = []

    for _ in range(n):
        mean_true: List[np.ndarray] = []
        mean_level: List[np.ndarray] = []
        for p in stations:
            los = los_to(truth, p)
            # 该站这一架次固定的零位偏置
            bx, by = rng.normal(0.0, bias, 2) if bias > 0 else (0.0, 0.0)
            base = _rotate_small(los, bx, by)
            jitter = [_rotate_small(base, *rng.normal(0.0, sigma, 2))
                      for _ in range(k)]
            m = mean_direction(jitter)
            mean_true.append(m)
            # 假定机体水平：丢掉真实倾角。倾角里有一份是"抵消水平外力"
            # 的固定量（一整个架次不变，平均不掉），另一份才是抖动。
            tx, ty = rng.normal(0.0, tilt_sigma_deg, 2)
            mean_level.append(_rotate_small(m, tx, ty + tilt_bias_deg))
        e = single_station_error(stations[0], mean_true[0], truth)
        if e is not None:
            single_err.append(e)
        e2 = single_station_error(stations[0], mean_level[0], truth)
        if e2 is not None:
            level_err.append(e2)
        if len(stations) > 1:
            f = from_station_directions(stations, mean_true, truth)
            if f["fused_m"] is not None:
                fused_err.append(float(f["fused_m"]))
        spreads.append(angular_spread_deg(mean_true))

    sens = [Ray(origin=p, direction=los_to(truth, p)).sensitivity_m_per_deg()
            for p in stations]
    out: Dict[str, object] = {
        "n_stations": len(stations),
        "intersection_deg": float(case.intersection_deg),
        "sigma_deg": sigma,
        "bias_deg": bias,
        "samples_per_station": k,
        "tilt_sigma_deg": float(tilt_sigma_deg),
        "tilt_bias_deg": round(float(tilt_bias_deg), 3),
        "sensitivity_m_per_deg": [round(float(s), 3) for s in sens],
        "los_spread_deg": float(np.mean(spreads)) if spreads else float("nan"),
        "single": summarize(single_err),
        "level_assumption": summarize(level_err),
    }
    out["fused"] = summarize(fused_err) if fused_err else None
    return out


def sweep_angle(angles: Sequence[float], sigma_deg: float, bias_deg: float,
                samples: int = 120, n: int = 300,
                seed: int = 1) -> Dict[str, List[float]]:
    """扫交会角：返回每档开角下的单站 / 交会 CEP95。"""
    from .cases import sweep_case

    out: Dict[str, List[float]] = {"angle_deg": [], "single_cep50_m": [],
                                   "single_cep95_m": [], "fused_cep50_m": [],
                                   "fused_cep95_m": []}
    for a in angles:
        case = sweep_case(float(a), sigma_deg=sigma_deg, bias_deg=bias_deg)
        res = monte_carlo(case, n=n, seed=seed,
                          samples_per_station=samples)
        out["angle_deg"].append(round(float(a), 2))
        out["single_cep50_m"].append(round(float(res["single"]["cep50_m"]), 4))
        out["single_cep95_m"].append(round(float(res["single"]["cep95_m"]), 4))
        f = res["fused"]
        out["fused_cep50_m"].append(round(float(f["cep50_m"]), 4) if f else float("nan"))
        out["fused_cep95_m"].append(round(float(f["cep95_m"]), 4) if f else float("nan"))
    return out


def theory_single_cep50(case: GeoCase, samples_per_station: Optional[int] = None) -> float:
    """单站定位的理论 CEP50：h/cos²θ · sqrt(bias² + (σ/√N)²) · 1.1774。"""
    k = int(case.samples if samples_per_station is None else samples_per_station)
    truth = np.asarray(case.target, dtype=float)
    p0 = case.stations[0].position
    sens = Ray(origin=p0, direction=los_to(truth, p0)).sensitivity_m_per_deg()
    ang = math.hypot(float(case.bias_deg), float(case.sigma_deg) / math.sqrt(k))
    # 二维各向同性 Rayleigh 分布的 CEP50 = 1.1774 σ
    return 1.1774 * sens * ang


def compare_to_theory(case: GeoCase, n: int = 600,
                      samples_per_station: Optional[int] = None) -> Dict[str, float]:
    """解析预期 vs 蒙特卡洛，测试里用它守住"数算得对"。"""
    mc = monte_carlo(case, n=n, seed=7,
                     samples_per_station=samples_per_station)
    exp = theory_single_cep50(case, samples_per_station)
    got = float(mc["single"]["cep50_m"])
    return {
        "expected_cep50_m": float(exp),
        "mc_cep50_m": got,
        "ratio": float(got / exp) if exp > 0 else float("inf"),
        "intersection_deg": float(case.intersection_deg),
        "pairwise_deg": float(pairwise_min_angle_deg(case.azimuths)),
    }
