"""Deterministic friction-scenario scheduling."""

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class FrictionScenario:
    """The active friction coefficient and zero-based scenario index."""

    index: int
    coefficient: float


class FrictionSchedule:
    """Cycle through positive friction coefficients using simulation time."""

    def __init__(self, coefficients: Sequence[float], duration_seconds: float):
        if not coefficients:
            raise ValueError('at least one friction coefficient is required')
        if not math.isfinite(duration_seconds) or duration_seconds <= 0.0:
            raise ValueError('duration_seconds must be a finite positive value')
        values = tuple(float(value) for value in coefficients)
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError('friction coefficients must be finite and positive')
        self._coefficients = values
        self._duration_seconds = duration_seconds

    def at(self, simulation_time: float) -> FrictionScenario:
        """Return the scenario selected at a non-negative simulation time."""
        if not math.isfinite(simulation_time) or simulation_time < 0.0:
            raise ValueError('simulation_time must be finite and non-negative')
        absolute_index = int(simulation_time // self._duration_seconds)
        index = absolute_index % len(self._coefficients)
        return FrictionScenario(index, self._coefficients[index])
