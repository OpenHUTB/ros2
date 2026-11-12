"""AirSim barometer sample normalization."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class BarometerSample:
    """Normalized AirSim barometer values."""

    altitude: float
    pressure: float
    qnh: float


def parse_barometer_sample(sample: Any) -> BarometerSample:
    """Validate and normalize an AirSim BarometerData sample."""
    altitude = float(sample.altitude)
    pressure = float(sample.pressure)
    qnh = float(sample.qnh)
    if not math.isfinite(altitude):
        raise ValueError("barometer altitude is not finite")
    if not math.isfinite(pressure) or pressure <= 0.0:
        raise ValueError("barometer pressure must be finite and positive")
    if not math.isfinite(qnh) or qnh <= 0.0:
        raise ValueError("barometer QNH must be finite and positive")
    return BarometerSample(
        altitude=altitude,
        pressure=pressure,
        qnh=qnh,
    )
