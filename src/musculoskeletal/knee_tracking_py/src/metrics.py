"""轨迹跟踪指标：RMSE、最大绝对误差、超调、调节时间、稳态误差。

口径与原报告保持一致：
* 正弦段参考持续变化，只报 RMSE 与最大绝对误差，不讨论超调。
* 阶跃超调相对“阶跃幅值”定义，即 (峰值 - 目标) / |目标 - 初值| * 100%。
* 调节时间使用 ±2% 带（2% * 阶跃幅值），且要求进入后一直保持。
* 另报“首次进入 ±5 deg 带”的时间，因为它反映上升速度。
* 稳态误差取最后 steady_window 秒的平均值与目标之差。
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np


def _as_arrays(t: Sequence[float], y: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    time = np.asarray(t, dtype=float)
    values = np.asarray(y, dtype=float)
    if time.ndim != 1 or values.ndim != 1:
        raise ValueError("t 与 y 必须是一维序列")
    if time.size == 0 or values.size == 0:
        raise ValueError("t 与 y 不能为空")
    if time.size != values.size:
        raise ValueError("t 与 y 长度必须一致")
    if not np.all(np.isfinite(time)) or not np.all(np.isfinite(values)):
        raise ValueError("t 与 y 不能包含 NaN/Inf")
    if time.size > 1 and np.any(np.diff(time) <= 0):
        raise ValueError("t 必须严格递增")
    return time, values


def rmse(actual: Sequence[float], reference: Sequence[float]) -> float:
    """均方根误差，单位与输入一致。"""
    a = np.asarray(actual, dtype=float)
    r = np.asarray(reference, dtype=float)
    if a.shape != r.shape:
        raise ValueError("actual 与 reference 形状必须一致")
    if a.size == 0:
        raise ValueError("输入不能为空")
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(r)):
        raise ValueError("输入不能包含 NaN/Inf")
    return float(np.sqrt(np.mean((a - r) ** 2)))


def max_abs_error(actual: Sequence[float], reference: Sequence[float]) -> float:
    """最大绝对跟踪误差。"""
    a = np.asarray(actual, dtype=float)
    r = np.asarray(reference, dtype=float)
    if a.shape != r.shape:
        raise ValueError("actual 与 reference 形状必须一致")
    if a.size == 0:
        raise ValueError("输入不能为空")
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(r)):
        raise ValueError("输入不能包含 NaN/Inf")
    return float(np.max(np.abs(a - r)))


def overshoot_percent(
    y: Sequence[float],
    target: float,
    step_amplitude_deg: float,
) -> float:
    """相对阶跃幅值的超调百分比；未越过目标时为 0。"""
    values = np.asarray(y, dtype=float)
    if values.size == 0:
        raise ValueError("y 不能为空")
    if not np.all(np.isfinite(values)):
        raise ValueError("y 不能包含 NaN/Inf")
    if not math.isfinite(target) or not math.isfinite(step_amplitude_deg):
        raise ValueError("target 与阶跃幅值必须有限")
    if step_amplitude_deg <= 0:
        raise ValueError("阶跃幅值必须 > 0")

    start = float(values[0])
    direction = 1.0 if target >= start else -1.0
    if direction > 0:
        peak = float(np.max(values))
        beyond = peak - target
    else:
        peak = float(np.min(values))
        beyond = target - peak
    return float(max(0.0, beyond) / step_amplitude_deg * 100.0)


def first_entry_time(
    t: Sequence[float],
    y: Sequence[float],
    target: float,
    band: float,
) -> float | None:
    """首次进入 |y-target| <= band 的时刻；从未进入返回 None。"""
    time, values = _as_arrays(t, y)
    if band < 0 or not math.isfinite(band):
        raise ValueError("band 必须是非负有限值")
    inside = np.abs(values - target) <= band
    if not bool(np.any(inside)):
        return None
    return float(time[int(np.argmax(inside))])


def settling_time(
    t: Sequence[float],
    y: Sequence[float],
    target: float,
    band: float,
) -> float | None:
    """进入并保持 |y-target| <= band 的时间；最终仍在带外则返回 None。"""
    time, values = _as_arrays(t, y)
    if band < 0 or not math.isfinite(band):
        raise ValueError("band 必须是非负有限值")
    outside = np.abs(values - target) > band
    if not bool(np.any(outside)):
        return float(time[0])
    last_outside = int(np.flatnonzero(outside)[-1])
    if last_outside == time.size - 1:
        return None
    return float(time[last_outside + 1])


def steady_state_error(
    t: Sequence[float],
    y: Sequence[float],
    target: float,
    window_s: float,
) -> float:
    """末段均值与目标之差。"""
    time, values = _as_arrays(t, y)
    if window_s <= 0 or not math.isfinite(window_s):
        raise ValueError("window_s 必须 > 0")
    mask = time >= time[-1] - window_s
    if not bool(np.any(mask)):
        raise ValueError("统计窗口内没有采样点")
    return float(np.mean(values[mask]) - target)


def sine_metrics(
    t: Sequence[float],
    actual: Sequence[float],
    reference: Sequence[float],
) -> dict[str, float]:
    """正弦段指标：RMSE 与最大绝对误差。"""
    _as_arrays(t, actual)
    if np.shape(reference) != np.shape(actual):
        raise ValueError("actual 与 reference 形状必须一致")
    return {
        "rmse_deg": rmse(actual, reference),
        "max_abs_error_deg": max_abs_error(actual, reference),
    }


def step_metrics(
    t: Sequence[float],
    actual: Sequence[float],
    target: float,
    step_amplitude_deg: float,
    band_percent: float = 2.0,
    wide_band_deg: float = 5.0,
    steady_window_s: float = 1.0,
) -> dict[str, float | bool | None]:
    """阶跃段指标。t 必须从阶跃时刻重新计时（t[0] = 0）。"""
    time, values = _as_arrays(t, actual)
    if abs(time[0]) > 1e-12:
        raise ValueError("阶跃指标要求 t[0] = 0（相对阶跃时刻）")
    if band_percent <= 0 or band_percent >= 100:
        raise ValueError("band_percent 必须在 (0, 100)")
    if wide_band_deg <= 0:
        raise ValueError("wide_band_deg 必须 > 0")
    band = band_percent / 100.0 * step_amplitude_deg
    settle = settling_time(time, values, target, band)
    wide_entry = first_entry_time(time, values, target, wide_band_deg)
    return {
        "overshoot_percent": overshoot_percent(values, target, step_amplitude_deg),
        "settling_time_s": settle,
        "settling_band_deg": float(band),
        "wide_band_entry_s": wide_entry,
        "steady_state_error_deg": steady_state_error(
            time, values, target, steady_window_s
        ),
        "peak_deg": float(np.max(values)),
        "final_deg": float(np.mean(values[time >= time[-1] - steady_window_s])),
    }
