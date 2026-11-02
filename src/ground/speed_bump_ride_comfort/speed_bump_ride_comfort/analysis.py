"""扫速、建议车速与演示数据集。

设计取舍：图里画的三条车速曲线、表格里的指标、以及"建议最高车速"都从同一份
扫速结果（``speed_curve``）推导，避免图形与文字结论不一致。
"""

from __future__ import annotations

import numpy as np

from .bump import SpeedBump
from .comfort import analysis_window, passage_metrics, peak_abs
from .vehicle import Passage, QuarterCar

#: 演示用的三条车速（m/s）：分别落在舒适阈值内、可接受区间内、以及车轮离地区间。
DEFAULT_SPEEDS_MPS = (1.5, 3.0, 8.0)
#: 车身垂向加速度峰值的两级阈值（m/s²）。仓库自定义的经验值，不是标准限值：
#: 超过"舒适"线约等于 0.3 g，超过"可接受"线约等于 0.6 g。
DEFAULT_LIMITS_MPS2 = (("舒适", 3.0), ("可接受", 6.0))
#: 主阈值（舒适线），用于指标表中的超限标记。
DEFAULT_LIMIT_MPS2 = DEFAULT_LIMITS_MPS2[0][1]
#: 扫速范围与步长（m/s）。下限取 1 m/s，让"低速可满足阈值"的部分落在网格内。
DEFAULT_SWEEP = {"v_min_mps": 1.0, "v_max_mps": 20.0, "step_mps": 0.5}


def sweep(
    vehicle: QuarterCar,
    bump: SpeedBump,
    speeds_mps,
    dt: float = 1.0e-3,
) -> list[Passage]:
    """按给定车速逐次仿真。"""
    return [vehicle.simulate(bump, float(speed), dt=dt) for speed in speeds_mps]


def speed_curve(
    vehicle: QuarterCar,
    bump: SpeedBump,
    dt: float = 1.0e-3,
    v_min_mps: float = 2.0,
    v_max_mps: float = 20.0,
    step_mps: float = 0.5,
) -> dict:
    """峰值加速度、最小轮荷比、最大动行程随车速的变化（未做四舍五入）。"""
    if not step_mps > 0.0:
        raise ValueError("step_mps must be positive")
    if not v_max_mps > v_min_mps:
        raise ValueError("v_max_mps must be greater than v_min_mps")
    count = int(np.floor((v_max_mps - v_min_mps) / step_mps + 1.0e-9)) + 1
    speeds = v_min_mps + step_mps * np.arange(count)
    peaks = np.zeros(count)
    min_load = np.zeros(count)
    travel = np.zeros(count)
    for index, speed in enumerate(speeds):
        passage = vehicle.simulate(bump, float(speed), dt=dt)
        mask = analysis_window(passage)
        peaks[index] = peak_abs(passage.sprung_accel_mps2[mask])
        min_load[index] = float(np.min(passage.tire_load_ratio[mask]))
        travel[index] = peak_abs(passage.travel_m[mask]) * 1000.0
    return {
        "speeds_mps": speeds,
        "peak_accel_mps2": peaks,
        "tire_load_min_ratio": min_load,
        "suspension_travel_mm": travel,
    }


def recommend_max_speed(curve: dict, limit_mps2: float) -> dict:
    """在扫速网格内取满足峰值阈值的最高车速。

    返回字典包含建议车速、该车速下的峰值、以及网格中下一个不满足阈值的车速
    （若存在），便于核对建议值与阈值的关系。
    """
    if not limit_mps2 > 0.0:
        raise ValueError("limit_mps2 must be positive")
    speeds = np.asarray(curve["speeds_mps"], dtype=float)
    peaks = np.asarray(curve["peak_accel_mps2"], dtype=float)
    accepted = peaks <= limit_mps2
    if not np.any(accepted):
        return {
            "speed_mps": None,
            "peak_accel_mps2": None,
            "first_speed_mps": float(speeds[0]),
            "first_peak_accel_mps2": float(peaks[0]),
            "limit_mps2": float(limit_mps2),
            "grid_step_mps": float(speeds[1] - speeds[0]) if speeds.size > 1 else 0.0,
        }
    index = int(np.where(accepted)[0].max())
    has_next = index + 1 < speeds.size
    return {
        "speed_mps": float(speeds[index]),
        "peak_accel_mps2": float(peaks[index]),
        "next_speed_mps": float(speeds[index + 1]) if has_next else None,
        "next_peak_accel_mps2": float(peaks[index + 1]) if has_next else None,
        "limit_mps2": float(limit_mps2),
        "grid_step_mps": float(speeds[1] - speeds[0]) if speeds.size > 1 else 0.0,
    }


def build_demo(
    vehicle: QuarterCar | None = None,
    bump: SpeedBump | None = None,
    speeds_mps=DEFAULT_SPEEDS_MPS,
    limits_mps2=DEFAULT_LIMITS_MPS2,
    dt: float = 1.0e-3,
    sweep_kwargs: dict | None = None,
) -> dict:
    """组装演示所需的全部数据：三条车速曲线、指标表、扫速曲线与各级建议车速。"""
    vehicle = vehicle or QuarterCar()
    bump = bump or SpeedBump()
    limits = tuple(limits_mps2)
    if not limits:
        raise ValueError("limits_mps2 must not be empty")
    passages = sweep(vehicle, bump, speeds_mps, dt=dt)
    primary_limit = float(limits[0][1])
    rows = [
        passage_metrics(
            passage,
            limit_mps2=primary_limit,
            travel_limit_mm=vehicle.travel_limit_m * 1000.0,
        )
        for passage in passages
    ]
    curve = speed_curve(vehicle, bump, dt=dt, **(sweep_kwargs or DEFAULT_SWEEP))
    advice = [
        {"label": str(label), **recommend_max_speed(curve, float(limit))}
        for label, limit in limits
    ]
    return {
        "vehicle": vehicle,
        "bump": bump,
        "dt_s": float(dt),
        "passages": passages,
        "rows": rows,
        "curve": curve,
        "advice": advice,
        "limits_mps2": [[str(label), float(limit)] for label, limit in limits],
        "primary_limit_mps2": primary_limit,
        "vehicle_summary": {
            "sprung_mass_kg": vehicle.sprung_mass_kg,
            "unsprung_mass_kg": vehicle.unsprung_mass_kg,
            "suspension_stiffness_n_per_m": vehicle.suspension_stiffness_n_per_m,
            "suspension_damping_n_s_per_m": vehicle.suspension_damping_n_s_per_m,
            "tire_stiffness_n_per_m": vehicle.tire_stiffness_n_per_m,
            "travel_limit_mm": vehicle.travel_limit_m * 1000.0,
            "body_natural_frequency_hz": round(vehicle.body_natural_frequency_hz, 4),
            "wheel_natural_frequency_hz": round(vehicle.wheel_natural_frequency_hz, 4),
            "damping_ratio": round(vehicle.damping_ratio, 4),
            "static_wheel_load_n": round(vehicle.static_wheel_load_n, 2),
        },
        "bump_summary": {
            "height_m": bump.height_m,
            "width_m": bump.width_m,
            "kind": bump.kind,
            "ramp_ratio": bump.ramp_ratio,
            "reference_area_m2": round(bump.reference_area_m2(), 6),
        },
    }
