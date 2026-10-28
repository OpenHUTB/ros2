"""Safety constraints shared by expert and learned controllers."""

from dataclasses import dataclass
from math import isfinite
from typing import Iterable, Tuple


@dataclass(frozen=True)
class ActivationLimiter:
    """Validate and clamp antagonist muscle activations."""

    lower: float = 0.0
    upper: float = 1.0

    def __post_init__(self) -> None:
        if not 0.0 <= self.lower < self.upper <= 1.0:
            raise ValueError("activation bounds must satisfy 0 <= lower < upper <= 1")

    def apply(self, values: Iterable[float]) -> Tuple[float, float]:
        values = tuple(float(value) for value in values)
        if len(values) != 2:
            raise ValueError("exactly two muscle activations are required")
        if not all(isfinite(value) for value in values):
            raise ValueError("muscle activations must be finite")
        return tuple(min(self.upper, max(self.lower, value)) for value in values)

    def safe_stop(self) -> Tuple[float, float]:
        """Return a deactivated command for shutdown and fault handling."""

        return 0.0, 0.0
