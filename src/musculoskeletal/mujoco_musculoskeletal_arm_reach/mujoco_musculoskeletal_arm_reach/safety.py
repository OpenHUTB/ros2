"""Shared validation for multi-muscle commands."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable, Tuple


@dataclass(frozen=True)
class ActivationLimiter:
    """Validate and clamp a fixed-length muscle activation command."""

    count: int
    lower: float = 0.0
    upper: float = 1.0

    def __post_init__(self) -> None:
        if self.count <= 0:
            raise ValueError('count must be positive')
        if not 0.0 <= self.lower < self.upper <= 1.0:
            raise ValueError(
                'activation bounds must satisfy 0 <= lower < upper <= 1'
            )

    def apply(self, values: Iterable[float]) -> Tuple[float, ...]:
        values = tuple(float(value) for value in values)
        if len(values) != self.count:
            raise ValueError(
                'expected %d muscle activations, got %d'
                % (self.count, len(values))
            )
        if not all(isfinite(value) for value in values):
            raise ValueError('muscle activations must be finite')
        return tuple(
            min(self.upper, max(self.lower, value)) for value in values
        )

    def safe_stop(self) -> Tuple[float, ...]:
        return (0.0,) * self.count
