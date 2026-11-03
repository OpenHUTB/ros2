"""两个与热衰退相关的工况：恒速长下坡与城市工况连续制动。

两个工况共用同一套热模型，指标口径一致，所以图、表、结论都来自同一次计算。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .disc import BrakeDisc, BrakeDiscSpec, FrictionFadeCurve
from .vehicle import (
    BrakingSpec,
    deceleration_mps2,
    per_disc_power_w,
    stop_energy_j,
    stopping_distance_m,
)

#: 长下坡工况：三条对比车速（km/h）、坡度、仿真时长。
DESCENT_SPEEDS_KMH: tuple[float, ...] = (40.0, 60.0, 80.0)
DESCENT_GRADE = 0.07
DESCENT_DURATION_S = 600.0
#: 城市工况：制动次数、起始车速、两次制动之间的冷却时间（对标制动衰退试验的短间隔循环）。
STOP_CYCLES = 16
STOP_SPEED_KMH = 100.0
STOP_COOLDOWN_S = 10.0
#: 指标里统一用这个车速评估"此刻紧急制动"的停车距离。
EMERGENCY_STOP_KMH = 100.0


def kmh_to_mps(speed_kmh: float) -> float:
    return float(speed_kmh) / 3.6


@dataclass
class DescentRun:
    """恒速下坡的温度历程与由此得到的能力指标。"""

    speed_kmh: float
    power_per_disc_w: float
    time_s: np.ndarray
    temperature_c: np.ndarray
    equilibrium_c: float | None
    fade_onset_s: float | None
    deceleration_mps2: np.ndarray
    stop_distance_m: float
    stop_distance_cold_m: float


@dataclass
class StopCyclesRun:
    """连续制动：每一次的起始温度、结束温度与停车距离。"""

    speed_kmh: float
    cooldown_s: float
    cycle: np.ndarray
    temperature_before_c: np.ndarray
    temperature_after_c: np.ndarray
    distance_m: np.ndarray
    distance_cold_m: float


@dataclass
class DemoBundle:
    disc: BrakeDisc
    braking: BrakingSpec
    descents: list[DescentRun] = field(default_factory=list)
    cycles: StopCyclesRun | None = None


def simulate_descent(
    disc: BrakeDisc,
    braking: BrakingSpec,
    speed_kmh: float,
    grade: float = DESCENT_GRADE,
    duration_s: float = DESCENT_DURATION_S,
    dt: float = 0.5,
) -> DescentRun:
    """恒速下坡：单盘输入功率恒定，积分盘温到 ``duration_s``。"""
    if not duration_s > 0.0:
        raise ValueError("duration_s must be positive")
    speed_mps = kmh_to_mps(speed_kmh)
    power = per_disc_power_w(speed_mps, grade, braking)
    steps = int(round(duration_s / dt)) + 1
    time = dt * np.arange(steps)
    temperature = np.empty(steps)
    temperature[0] = disc.spec.ambient_c
    for index in range(1, steps):
        temperature[index] = disc.step(temperature[index - 1], power, dt)

    curve = disc.curve
    decel = np.array(
        [deceleration_mps2(t, braking, curve) for t in temperature]
    )
    emergency_speed = kmh_to_mps(EMERGENCY_STOP_KMH)
    return DescentRun(
        speed_kmh=float(speed_kmh),
        power_per_disc_w=power,
        time_s=time,
        temperature_c=temperature,
        equilibrium_c=disc.equilibrium_temperature_c(power),
        fade_onset_s=disc.time_to_reach_c(power, curve.fade_start_c, dt=dt),
        deceleration_mps2=decel,
        stop_distance_m=stopping_distance_m(
            emergency_speed, float(temperature[-1]), braking, curve
        ),
        stop_distance_cold_m=stopping_distance_m(
            emergency_speed, disc.spec.ambient_c, braking, curve
        ),
    )


def simulate_stop_cycles(
    disc: BrakeDisc,
    braking: BrakingSpec,
    speed_kmh: float = STOP_SPEED_KMH,
    cycles: int = STOP_CYCLES,
    cooldown_s: float = STOP_COOLDOWN_S,
    dt: float = 0.05,
) -> StopCyclesRun:
    """城市工况：急刹到停 → 冷却 ``cooldown_s`` → 再急刹，共 ``cycles`` 次。

    单次制动把整车动能平均分给四只制动盘，并按"制动时长内功率恒定"积分盘温；
    冷却段输入功率为零。
    """
    if cycles < 1:
        raise ValueError("cycles must be at least 1")
    if cooldown_s < 0.0:
        raise ValueError("cooldown_s must not be negative")
    speed_mps = kmh_to_mps(speed_kmh)
    energy_per_disc = stop_energy_j(speed_mps, braking) / braking.brake_count
    curve = disc.curve

    before = np.empty(cycles)
    after = np.empty(cycles)
    distance = np.empty(cycles)
    temperature = disc.spec.ambient_c
    for index in range(cycles):
        before[index] = temperature
        distance[index] = stopping_distance_m(speed_mps, temperature, braking, curve)
        deceleration = deceleration_mps2(temperature, braking, curve)
        braking_time = speed_mps / deceleration if deceleration > 0.0 else float("inf")
        if not np.isfinite(braking_time):
            after[index] = temperature
            continue
        power = energy_per_disc / braking_time
        steps = max(1, int(round(braking_time / dt)))
        step_dt = braking_time / steps
        for _ in range(steps):
            temperature = disc.step(temperature, power, step_dt)
        after[index] = temperature
        if index + 1 < cycles:
            cool_steps = max(0, int(round(cooldown_s / dt)))
            for _ in range(cool_steps):
                temperature = disc.step(temperature, 0.0, dt)
    return StopCyclesRun(
        speed_kmh=float(speed_kmh),
        cooldown_s=float(cooldown_s),
        cycle=np.arange(1, cycles + 1),
        temperature_before_c=before,
        temperature_after_c=after,
        distance_m=distance,
        distance_cold_m=stopping_distance_m(
            speed_mps, disc.spec.ambient_c, braking, curve
        ),
    )


def build_demo(
    disc: BrakeDisc | None = None,
    braking: BrakingSpec | None = None,
    speeds_kmh=DESCENT_SPEEDS_KMH,
    grade: float = DESCENT_GRADE,
    duration_s: float = DESCENT_DURATION_S,
    cycles: int = STOP_CYCLES,
    cooldown_s: float = STOP_COOLDOWN_S,
) -> DemoBundle:
    """组装演示数据：三条车速的长下坡历程 + 一组连续制动。"""
    disc = disc or BrakeDisc()
    braking = braking or BrakingSpec()
    bundle = DemoBundle(disc=disc, braking=braking)
    bundle.descents = [
        simulate_descent(disc, braking, speed, grade=grade, duration_s=duration_s)
        for speed in speeds_kmh
    ]
    bundle.cycles = simulate_stop_cycles(
        disc, braking, cycles=cycles, cooldown_s=cooldown_s
    )
    return bundle


def summary(bundle: DemoBundle) -> dict:
    """把演示数据整理成可写进 JSON 的摘要。"""
    disc, braking = bundle.disc, bundle.braking
    descents = [
        {
            "speed_kmh": run.speed_kmh,
            "power_per_disc_w": round(run.power_per_disc_w, 1),
            "equilibrium_c": None if run.equilibrium_c is None else round(run.equilibrium_c, 1),
            "fade_onset_s": None if run.fade_onset_s is None else round(run.fade_onset_s, 1),
            "final_temperature_c": round(float(run.temperature_c[-1]), 1),
            "stop_distance_m": round(run.stop_distance_m, 1),
            "stop_distance_cold_m": round(run.stop_distance_cold_m, 1),
        }
        for run in bundle.descents
    ]
    cycles = bundle.cycles
    cycle_summary = None
    if cycles is not None:
        cycle_summary = {
            "speed_kmh": cycles.speed_kmh,
            "cooldown_s": cycles.cooldown_s,
            "cycles": int(cycles.cycle.size),
            "temperature_max_c": round(float(cycles.temperature_after_c.max()), 1),
            "distance_first_m": round(float(cycles.distance_m[0]), 1),
            "distance_last_m": round(float(cycles.distance_m[-1]), 1),
            "distance_cold_m": round(cycles.distance_cold_m, 1),
            "per_cycle": [
                {
                    "cycle": int(index),
                    "temperature_before_c": round(float(before), 1),
                    "temperature_after_c": round(float(after), 1),
                    "distance_m": round(float(distance), 1),
                }
                for index, before, after, distance in zip(
                    cycles.cycle,
                    cycles.temperature_before_c,
                    cycles.temperature_after_c,
                    cycles.distance_m,
                )
            ],
        }
    return {
        "disc": {
            "disc_mass_kg": disc.spec.disc_mass_kg,
            "specific_heat_j_per_kgk": disc.spec.specific_heat_j_per_kgk,
            "heat_capacity_j_per_k": disc.spec.heat_capacity_j_per_k,
            "cooling_w_per_k": disc.spec.cooling_w_per_k,
            "radiation_area_m2": disc.spec.radiation_area_m2,
            "emissivity": disc.spec.emissivity,
            "ambient_c": disc.spec.ambient_c,
        },
        "friction": {
            "mu_cold": disc.curve.mu_cold,
            "fade_start_c": disc.curve.fade_start_c,
            "fade_end_c": disc.curve.fade_end_c,
            "mu_hot": disc.curve.mu_hot,
        },
        "vehicle": {
            "mass_kg": braking.mass_kg,
            "reference_decel_mps2": braking.reference_decel_mps2,
            "brake_count": braking.brake_count,
        },
        "grade": DESCENT_GRADE,
        "descent_duration_s": DESCENT_DURATION_S,
        "descents": descents,
        "stop_cycles": cycle_summary,
    }
