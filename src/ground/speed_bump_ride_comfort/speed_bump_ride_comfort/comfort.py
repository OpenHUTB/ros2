"""平顺性指标：峰值、有效值、振动剂量值与轮荷/行程极值。

所有指标都给出明确定义与单位，避免把不同来源的"舒适性等级"混用：

* 峰值加速度 ``max |a_s|``，单位 m/s²，反映瞬间冲击强度；
* 有效值 ``a_rms = sqrt(mean(a_s²))``，单位 m/s²；
* 振动剂量值 ``VDV = (∫ a_s⁴ dt)^(1/4)``，单位 m/s^1.75，对冲击峰值更敏感；
* 轮胎动载比 ``F_z / F_z,static``，小于等于 0 表示车轮离地；
* 悬挂动行程 ``max |z_s - z_u|``，单位 mm，用于判断是否触及限位。

注意：这里没有实现 ISO 2631 的频率加权滤波器（Wk），因此结果不与标准限值直接
比较；判断是否"建议减速"用的是本仓库自定义的峰值阈值，见 ``analysis.py``。
"""

from __future__ import annotations

import numpy as np

from .vehicle import Passage


def peak_abs(values: np.ndarray) -> float:
    """绝对值的最大值。"""
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("values must not be empty")
    return float(np.max(np.abs(array)))


def rms(values: np.ndarray) -> float:
    """均方根值 ``sqrt(mean(x²))``。"""
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("values must not be empty")
    return float(np.sqrt(np.mean(np.square(array))))


def vibration_dose_value(accel: np.ndarray, dt: float) -> float:
    """振动剂量值 ``(∫ a⁴ dt)^(1/4)``，输入为加速度（m/s²）与采样间隔（s）。

    积分用梯形法，常数加速度 ``a`` 持续 ``T`` 时结果恰好是 ``a T^(1/4)``。
    """
    array = np.asarray(accel, dtype=float)
    if array.size == 0:
        raise ValueError("accel must not be empty")
    if not dt > 0.0:
        raise ValueError("dt must be positive")
    if array.size < 2:
        raise ValueError("accel must contain at least two samples")
    integral = float(np.trapezoid(np.power(array, 4.0), dx=float(dt)))
    return float(np.power(integral, 0.25))


def analysis_window(passage: Passage) -> np.ndarray:
    """分析窗口掩码：车轮接触减速带起（``t >= 0``）到仿真结束（含回落段）。"""
    return passage.time_s >= 0.0


def passage_metrics(
    passage: Passage,
    limit_mps2: float = 3.0,
    travel_limit_mm: float | None = None,
) -> dict:
    """把一次通过整理成指标字典，键名与单位一并给出。

    ``travel_limit_mm`` 给出时额外判断悬挂动行程是否超过限位。
    """
    mask = analysis_window(passage)
    accel = passage.sprung_accel_mps2[mask]
    time = passage.time_s[mask]
    dt = float(time[1] - time[0]) if time.size > 1 else 1.0e-3
    travel_mm = peak_abs(passage.travel_m[mask]) * 1000.0
    peak = peak_abs(accel)
    metrics = {
        "speed_mps": round(float(passage.speed_mps), 3),
        "speed_kmh": round(float(passage.speed_mps) * 3.6, 2),
        "peak_accel_mps2": round(peak, 4),
        "rms_accel_mps2": round(rms(accel), 4),
        "vdv_mps1_75": round(vibration_dose_value(accel, dt), 4),
        "tire_load_min_ratio": round(float(np.min(passage.tire_load_ratio[mask])), 4),
        "tire_load_peak_ratio": round(float(np.max(passage.tire_load_ratio[mask])), 4),
        "suspension_travel_mm": round(travel_mm, 3),
        "comfort_limit_mps2": round(float(limit_mps2), 3),
        "exceeds_comfort_limit": bool(peak > limit_mps2),
    }
    metrics["exceeds_travel_limit"] = (
        None if travel_limit_mm is None else bool(travel_mm > travel_limit_mm)
    )
    return metrics
