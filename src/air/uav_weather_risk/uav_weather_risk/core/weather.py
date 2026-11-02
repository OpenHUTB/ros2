"""Multi-source wind fusion and a discrete Dryden-style gust surrogate."""

from __future__ import annotations

import math
import random
from statistics import fmean
from typing import Iterable

from .types import FusedWeather, Vec3, WeatherSource, vnorm, vscale, vsub, weighted_sum


def fuse_sources(
    sources: Iterable[WeatherSource],
    *,
    age_tau_s: float = 30.0,
    min_confidence: float = 0.02,
) -> FusedWeather:
    """Fuse wind estimates with inverse-variance weights.

    Weight = confidence / (sigma^2 * (1 + age / age_tau)). The fused standard
    uncertainty follows first-order propagation of independent estimates:
    sigma_fused = sqrt(sum(w_i^2 * sigma_i^2)) / sum(w_i).
    """

    items = tuple(sources)
    if not items:
        raise ValueError("at least one weather source is required")
    if age_tau_s <= 0.0:
        raise ValueError("age_tau_s must be positive")

    weighted: list[tuple[WeatherSource, float]] = []
    for source in items:
        if source.confidence < min_confidence:
            continue
        age_penalty = 1.0 + source.age_s / age_tau_s
        weight = source.confidence / (source.sigma_mps**2 * age_penalty)
        weighted.append((source, weight))

    if not weighted:
        usable = [s for s in items if s.confidence > 0.0]
        if not usable:
            raise ValueError("all weather sources have zero confidence")
        weighted = [(source, 1.0 / source.sigma_mps**2) for source in usable]

    total = sum(weight for _, weight in weighted)
    fused = weighted_sum((weight / total, source.wind_enu) for source, weight in weighted)
    variance = sum((weight / total) ** 2 * source.sigma_mps**2 for source, weight in weighted)
    effective = total * total / sum(weight * weight for _, weight in weighted)
    return FusedWeather(
        wind_enu=fused,
        sigma_mps=math.sqrt(max(variance, 0.0)),
        sources=items,
        effective_samples=effective,
    )


class GustProcess:
    """Stationary 3-D Ornstein-Uhlenbeck process used as a gust surrogate.

    The continuous process has correlation time ``tau_s`` and standard
    deviation ``sigma_mps``. The vertical component is scaled down because the
    demo represents a small multirotor near 30 m AGL.
    """

    def __init__(
        self,
        sigma_mps: float,
        tau_s: float,
        rng: random.Random | None = None,
        *,
        vertical_ratio: float = 0.6,
    ) -> None:
        if not math.isfinite(sigma_mps) or sigma_mps < 0.0:
            raise ValueError("sigma_mps must be finite and non-negative")
        if not math.isfinite(tau_s) or tau_s <= 0.0:
            raise ValueError("tau_s must be finite and positive")
        if not 0.0 <= vertical_ratio <= 1.0:
            raise ValueError("vertical_ratio must be within [0, 1]")
        self.sigma_mps = sigma_mps
        self.tau_s = tau_s
        self.vertical_ratio = vertical_ratio
        self._rng = rng or random.Random()
        self._value: Vec3 = (0.0, 0.0, 0.0)

    @property
    def value(self) -> Vec3:
        return self._value

    def step(self, dt_s: float) -> Vec3:
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        decay = math.exp(-dt_s / self.tau_s)
        driving = self.sigma_mps * math.sqrt(max(0.0, 1.0 - decay * decay))
        self._value = (
            decay * self._value[0] + driving * self._rng.gauss(0.0, 1.0),
            decay * self._value[1] + driving * self._rng.gauss(0.0, 1.0),
            decay * self._value[2] + driving * self.vertical_ratio * self._rng.gauss(0.0, 1.0),
        )
        return self._value


def source_spread(sources: Iterable[WeatherSource]) -> float:
    """Return mean pairwise disagreement in wind speed, useful for diagnostics."""

    items = tuple(sources)
    if len(items) < 2:
        return 0.0
    means = [fmean([v[i] for v in (s.wind_enu for s in items)]) for i in range(3)]
    return fmean(vnorm(vsub(s.wind_enu, (means[0], means[1], means[2]))) for s in items)


def gust_intensity(sources: Iterable[WeatherSource]) -> float:
    """A simple 0-1 style spread indicator for UI purposes."""

    return min(1.0, source_spread(sources) / 4.0)
