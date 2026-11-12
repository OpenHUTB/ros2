"""Small vector helpers and shared data containers.

All vectors use a local ENU frame: x east, y north, z up. Wind vectors describe
the air velocity relative to the ground, in metres per second.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite
from typing import Iterable, Sequence

Vec3 = tuple[float, float, float]


def vec3(values: Sequence[float]) -> Vec3:
    if len(values) != 3:
        raise ValueError("expected a three-element vector")
    out = (float(values[0]), float(values[1]), float(values[2]))
    if not all(isfinite(v) for v in out):
        raise ValueError("vector contains a non-finite value")
    return out


def vadd(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vscale(a: Vec3, factor: float) -> Vec3:
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def vnorm(a: Vec3) -> float:
    return hypot(hypot(a[0], a[1]), a[2])


def vclip(a: Vec3, limit: float) -> Vec3:
    if limit <= 0.0:
        raise ValueError("limit must be positive")
    norm = vnorm(a)
    if norm <= limit or norm == 0.0:
        return a
    return vscale(a, limit / norm)


def weighted_sum(items: Iterable[tuple[float, Vec3]]) -> Vec3:
    x = y = z = 0.0
    for weight, value in items:
        x += weight * value[0]
        y += weight * value[1]
        z += weight * value[2]
    return (x, y, z)


@dataclass(frozen=True)
class WeatherSource:
    """One wind estimate and its declared uncertainty."""

    source_id: str
    kind: str
    wind_enu: Vec3
    sigma_mps: float
    confidence: float = 1.0
    age_s: float = 0.0

    def __post_init__(self) -> None:
        if not self.source_id:
            raise ValueError("source_id must not be empty")
        if not isfinite(self.sigma_mps) or self.sigma_mps <= 0.0:
            raise ValueError("sigma_mps must be finite and positive")
        if not isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be within [0, 1]")
        if not isfinite(self.age_s) or self.age_s < 0.0:
            raise ValueError("age_s must be finite and non-negative")
        vec3(self.wind_enu)

    def as_dict(self) -> dict:
        return {
            "source_id": self.source_id,
            "kind": self.kind,
            "wind_enu": list(self.wind_enu),
            "sigma_mps": self.sigma_mps,
            "confidence": self.confidence,
            "age_s": self.age_s,
        }


@dataclass(frozen=True)
class FusedWeather:
    wind_enu: Vec3
    sigma_mps: float
    sources: tuple[WeatherSource, ...]
    effective_samples: float

    def as_dict(self) -> dict:
        return {
            "wind_enu": list(self.wind_enu),
            "sigma_mps": self.sigma_mps,
            "effective_samples": self.effective_samples,
            "sources": [source.as_dict() for source in self.sources],
        }
