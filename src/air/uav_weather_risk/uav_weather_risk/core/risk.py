"""Monte-Carlo short-horizon risk assessment."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Iterable

from .aircraft import AircraftModel, AircraftParams, AircraftState
from .types import Vec3, vadd, vnorm, vsub
from .weather import GustProcess

LEVEL_NAMES = ("NOMINAL", "CAUTION", "WARNING", "CRITICAL")


@dataclass(frozen=True)
class RiskThresholds:
    horizon_s: float = 10.0
    dt_s: float = 0.25
    samples: int = 64
    max_wind_mps: float = 11.0
    max_tilt_deg: float = 33.0
    max_position_error_m: float = 2.5
    max_control_saturation: float = 1.10
    turbulence_sigma_mps: float = 1.3
    turbulence_tau_s: float = 3.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.horizon_s) or self.horizon_s <= 0.0:
            raise ValueError("horizon_s must be finite and positive")
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        if self.samples < 4:
            raise ValueError("samples must be at least 4")
        for name in (
            "max_wind_mps",
            "max_tilt_deg",
            "max_position_error_m",
            "max_control_saturation",
            "turbulence_sigma_mps",
            "turbulence_tau_s",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class RiskResult:
    level: int
    level_name: str
    score: float
    probabilities: dict[str, float]
    predicted_p95: dict[str, float]
    dominant_factor: str
    reason: str
    horizon_s: float
    samples: int
    paths: tuple[tuple[Vec3, ...], ...] = ()

    def as_dict(self) -> dict:
        return {
            "level": self.level,
            "level_name": self.level_name,
            "score": self.score,
            "probabilities": dict(self.probabilities),
            "predicted_p95": dict(self.predicted_p95),
            "dominant_factor": self.dominant_factor,
            "reason": self.reason,
            "horizon_s": self.horizon_s,
            "samples": self.samples,
        }


def classify_risk(score: float, probabilities: dict[str, float]) -> tuple[int, str]:
    """Map a weighted score and worst probability to a four-level advisory."""

    if not 0.0 <= score <= 1.0:
        raise ValueError("score must be within [0, 1]")
    if any(not 0.0 <= value <= 1.0 for value in probabilities.values()):
        raise ValueError("probabilities must be within [0, 1]")
    worst = max(probabilities.values(), default=0.0)
    if score >= 0.70 or worst >= 0.98:
        level = 3
    elif score >= 0.45 or worst >= 0.90:
        level = 2
    elif score >= 0.20 or worst >= 0.50:
        level = 1
    else:
        level = 0
    return level, LEVEL_NAMES[level]


def _p95(values: Iterable[float]) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    index = min(len(ordered) - 1, max(0, math.ceil(0.95 * len(ordered)) - 1))
    return ordered[index]


def assess_risk(
    state: AircraftState,
    mean_wind_enu: Vec3,
    target_enu: Vec3,
    thresholds: RiskThresholds | None = None,
    *,
    params: AircraftParams | None = None,
    seed: int = 0,
    keep_paths: int = 0,
) -> RiskResult:
    """Roll out an ensemble under stochastic gusts and score exceedance risk.

    The prediction assumes the fused mean wind is constant over the horizon and
    gusts follow the configured Ornstein-Uhlenbeck process. This is an advisory
    surrogate, not a probabilistic weather forecast.
    """

    thr = thresholds or RiskThresholds()
    vehicle = params or AircraftParams()
    if keep_paths < 0:
        raise ValueError("keep_paths must be non-negative")
    rng = random.Random(seed)
    steps = max(1, int(round(thr.horizon_s / thr.dt_s)))
    max_winds: list[float] = []
    max_tilts: list[float] = []
    max_errors: list[float] = []
    max_saturations: list[float] = []
    paths: list[tuple[Vec3, ...]] = []
    path_stride = max(1, thr.samples // keep_paths) if keep_paths else 0
    reference_path: list[Vec3] = []
    reference = AircraftModel(vehicle, state)
    for _ in range(steps):
        reference_state = reference.step(thr.dt_s, (0.0, 0.0, 0.0), target_enu)
        reference_path.append(reference_state.position_enu)

    for sample_index in range(thr.samples):
        model = AircraftModel(vehicle, state)
        gust = GustProcess(thr.turbulence_sigma_mps, thr.turbulence_tau_s, rng)
        max_wind = 0.0
        max_tilt = 0.0
        max_error = 0.0
        max_saturation = 0.0
        path: list[Vec3] | None = [] if keep_paths and sample_index % path_stride == 0 else None
        if path is not None:
            path.append(state.position_enu)
        for step_index in range(steps):
            wind = vadd(mean_wind_enu, gust.step(thr.dt_s))
            current = model.step(thr.dt_s, wind, target_enu)
            max_wind = max(max_wind, vnorm(wind))
            max_tilt = max(max_tilt, current.tilt_deg)
            max_error = max(
                max_error,
                vnorm(vsub(reference_path[step_index], current.position_enu)),
            )
            max_saturation = max(max_saturation, current.control_saturation)
            if path is not None:
                path.append(current.position_enu)
        max_winds.append(max_wind)
        max_tilts.append(max_tilt)
        max_errors.append(max_error)
        max_saturations.append(max_saturation)
        if path is not None:
            paths.append(tuple(path))

    probabilities = {
        "wind": sum(v > thr.max_wind_mps for v in max_winds) / thr.samples,
        "tilt": sum(v > thr.max_tilt_deg for v in max_tilts) / thr.samples,
        "position": sum(v > thr.max_position_error_m for v in max_errors) / thr.samples,
        "control": sum(v > thr.max_control_saturation for v in max_saturations) / thr.samples,
    }
    weights = {"wind": 0.15, "tilt": 0.35, "position": 0.25, "control": 0.25}
    score = sum(weights[name] * value for name, value in probabilities.items())
    level, name = classify_risk(score, probabilities)
    dominant = max(probabilities, key=probabilities.get)
    labels = {
        "wind": f"wind above {thr.max_wind_mps:.1f} m/s",
        "tilt": f"tilt above {thr.max_tilt_deg:.1f} deg",
        "position": f"position error above {thr.max_position_error_m:.1f} m",
        "control": f"control saturation above {thr.max_control_saturation:.2f}",
    }
    reason = (
        f"{labels[dominant]} in {probabilities[dominant] * 100:.0f}% of "
        f"{thr.samples} rollouts over {thr.horizon_s:.1f} s"
    )
    return RiskResult(
        level=level,
        level_name=name,
        score=score,
        probabilities=probabilities,
        predicted_p95={
            "wind_mps": _p95(max_winds),
            "tilt_deg": _p95(max_tilts),
            "position_error_m": _p95(max_errors),
            "control_saturation": _p95(max_saturations),
        },
        dominant_factor=dominant,
        reason=reason,
        horizon_s=thr.horizon_s,
        samples=thr.samples,
        paths=tuple(paths),
    )
