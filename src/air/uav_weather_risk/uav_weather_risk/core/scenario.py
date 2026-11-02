"""Deterministic synthetic scenario used by the demo and regression tests."""

from __future__ import annotations

import hashlib
import json
import math
import random
from dataclasses import dataclass
from typing import Sequence

from .aircraft import AircraftModel, AircraftParams, AircraftState
from .risk import RiskThresholds, assess_risk
from .types import Vec3, WeatherSource, vadd, vnorm, vscale, vsub
from .weather import GustProcess, fuse_sources

DEFAULT_ROUTE: tuple[Vec3, ...] = (
    (30.0, 0.0, 30.0),
    (30.0, 30.0, 30.0),
    (0.0, 30.0, 30.0),
    (0.0, 0.0, 30.0),
)


def _smoothstep(edge0: float, edge1: float, value: float) -> float:
    if edge0 == edge1:
        return 1.0 if value >= edge1 else 0.0
    x = min(1.0, max(0.0, (value - edge0) / (edge1 - edge0)))
    return x * x * (3.0 - 2.0 * x)


class GustFrontScenario:
    """A compact gust-front scenario with three asynchronous wind sources."""

    base_wind: Vec3 = (4.0, 1.0, 0.0)
    front_wind: Vec3 = (7.0, 2.0, -3.0)

    def __init__(self, seed: int = 42) -> None:
        self._rng = random.Random(seed)
        self._unit_gust = GustProcess(1.0, 3.0, self._rng)
        self._history: list[tuple[float, Vec3]] = [(0.0, self.base_wind)]
        self.time_s = 0.0
        self.true_wind: Vec3 = self.base_wind

    def envelope(self, time_s: float) -> float:
        return _smoothstep(10.0, 20.0, time_s) * (1.0 - _smoothstep(38.0, 50.0, time_s))

    def mean_wind(self, time_s: float) -> Vec3:
        env = self.envelope(time_s)
        return vadd(self.base_wind, vscale(self.front_wind, env))

    def step(self, dt_s: float) -> Vec3:
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        self.time_s += dt_s
        env = self.envelope(self.time_s)
        gust_sigma = 0.55 + 1.15 * env
        gust = vscale(self._unit_gust.step(dt_s), gust_sigma)
        self.true_wind = vadd(self.mean_wind(self.time_s), gust)
        self._history.append((self.time_s, self.true_wind))
        if len(self._history) > 4000:
            self._history = self._history[-2000:]
        return self.true_wind

    def _delayed(self, delay_s: float) -> Vec3:
        target = self.time_s - delay_s
        for stamp, wind in reversed(self._history):
            if stamp <= target:
                return wind
        return self._history[0][1]

    def sources(self, time_s: float | None = None) -> tuple[WeatherSource, ...]:
        t = self.time_s if time_s is None else time_s
        phase = 0.35 * math.sin(0.61 * t)
        nwp = vscale(self.mean_wind(max(0.0, t - 6.0)), 0.85)
        nwp = (nwp[0] + phase, nwp[1] - 0.4 * phase, nwp[2])
        ground = self._delayed(0.2)
        ground = (
            ground[0] + 0.22 * math.sin(1.7 * t),
            ground[1] + 0.18 * math.cos(1.3 * t),
            ground[2] + 0.12 * math.sin(2.1 * t),
        )
        onboard = self._delayed(1.0)
        onboard = (onboard[0] - 0.3, onboard[1] + 0.2, onboard[2] + 0.1)
        return (
            WeatherSource("nwp_meso", "nwp", nwp, 1.4, 0.65, 6.0),
            WeatherSource("ground_mast", "in_situ", ground, 0.7, 0.90, 0.2),
            WeatherSource("onboard_estimator", "onboard", onboard, 0.9, 0.85, 1.0),
        )


@dataclass(frozen=True)
class DemoResult:
    records: tuple[dict, ...]
    summary: dict
    thresholds: RiskThresholds


def run_demo(
    *,
    duration_s: float = 60.0,
    dt_s: float = 0.1,
    assessment_period_s: float = 0.5,
    seed: int = 42,
    route: Sequence[Vec3] = DEFAULT_ROUTE,
    start_enu: Vec3 = (0.0, 0.0, 30.0),
    thresholds: RiskThresholds | None = None,
    params: AircraftParams | None = None,
) -> DemoResult:
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be finite and positive")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    if not math.isfinite(assessment_period_s) or assessment_period_s < dt_s:
        raise ValueError("assessment_period_s must be at least dt_s")
    if not route:
        raise ValueError("route must not be empty")

    thr = thresholds or RiskThresholds()
    vehicle_params = params or AircraftParams()
    scenario = GustFrontScenario(seed)
    vehicle = AircraftModel(vehicle_params, AircraftState(position_enu=start_enu))
    records: list[dict] = []
    target_index = 0
    target = route[0]
    next_assessment = 0.0
    assessment_index = 0
    steps = int(round(duration_s / dt_s))
    max_true_wind = max_fused_wind = 0.0
    max_tilt = max_saturation = max_score = 0.0
    level_counts = {name: 0 for name in ("NOMINAL", "CAUTION", "WARNING", "CRITICAL")}
    peak_time = 0.0

    for step_index in range(steps + 1):
        time_s = step_index * dt_s
        if step_index > 0:
            true_wind = scenario.step(dt_s)
            vehicle.step(dt_s, true_wind, target)
        else:
            true_wind = scenario.true_wind
        sources = scenario.sources(time_s)
        fused = fuse_sources(sources)
        state = vehicle.state
        target_error = vnorm(vsub(target, state.position_enu))
        if target_error <= 2.0:
            target_index = (target_index + 1) % len(route)
            target = route[target_index]

        if time_s + 1e-9 >= next_assessment:
            result = assess_risk(
                state,
                fused.wind_enu,
                target,
                thr,
                params=vehicle_params,
                seed=seed + assessment_index * 1009,
            )
            fused_speed = vnorm(fused.wind_enu)
            record = {
                "t": time_s,
                "x": state.position_enu[0],
                "y": state.position_enu[1],
                "z": state.position_enu[2],
                "vx": state.velocity_enu[0],
                "vy": state.velocity_enu[1],
                "vz": state.velocity_enu[2],
                "target_x": target[0],
                "target_y": target[1],
                "target_z": target[2],
                "true_wind_x": true_wind[0],
                "true_wind_y": true_wind[1],
                "true_wind_z": true_wind[2],
                "fused_wind_x": fused.wind_enu[0],
                "fused_wind_y": fused.wind_enu[1],
                "fused_wind_z": fused.wind_enu[2],
                "true_wind_speed": vnorm(true_wind),
                "fused_wind_speed": fused_speed,
                "fused_sigma": fused.sigma_mps,
                "nwp_speed": vnorm(sources[0].wind_enu),
                "ground_speed": vnorm(sources[1].wind_enu),
                "onboard_speed": vnorm(sources[2].wind_enu),
                "tilt_deg": state.tilt_deg,
                "control_saturation": state.control_saturation,
                "risk_level": result.level,
                "risk_name": result.level_name,
                "risk_score": result.score,
                "p_wind": result.probabilities["wind"],
                "p_tilt": result.probabilities["tilt"],
                "p_position": result.probabilities["position"],
                "p_control": result.probabilities["control"],
                "p95_wind": result.predicted_p95["wind_mps"],
                "p95_tilt": result.predicted_p95["tilt_deg"],
                "reason": result.reason,
            }
            records.append(record)
            level_counts[result.level_name] += 1
            max_true_wind = max(max_true_wind, record["true_wind_speed"])
            max_fused_wind = max(max_fused_wind, fused_speed)
            max_tilt = max(max_tilt, state.tilt_deg)
            max_saturation = max(max_saturation, state.control_saturation)
            if result.score > max_score:
                max_score = result.score
                peak_time = time_s
            assessment_index += 1
            next_assessment += assessment_period_s

    canonical = json.dumps(records, sort_keys=True, separators=(",", ":")).encode("utf-8")
    summary = {
        "seed": seed,
        "duration_s": duration_s,
        "dt_s": dt_s,
        "assessment_period_s": assessment_period_s,
        "assessments": len(records),
        "samples_per_assessment": thr.samples,
        "max_true_wind_mps": max_true_wind,
        "max_fused_wind_mps": max_fused_wind,
        "max_tilt_deg": max_tilt,
        "max_control_saturation": max_saturation,
        "max_risk_score": max_score,
        "peak_risk_time_s": peak_time,
        "level_counts": level_counts,
        "record_sha256": hashlib.sha256(canonical).hexdigest(),
    }
    return DemoResult(tuple(records), summary, thr)
