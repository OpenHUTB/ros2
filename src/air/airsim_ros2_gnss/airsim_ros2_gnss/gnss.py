"""AirSim GNSS parsing and ROS NavSatFix value conversion."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from sensor_msgs.msg import NavSatStatus


@dataclass(frozen=True)
class GnssSample:
    """Normalized AirSim GNSS values used by the ROS node."""

    latitude: float
    longitude: float
    altitude: float
    valid: bool
    fix_type: int
    horizontal_accuracy: float
    vertical_accuracy: float


def _first_float(source: Any, names: tuple[str, ...], default: float) -> float:
    for name in names:
        if not hasattr(source, name):
            continue
        value = float(getattr(source, name))
        if not math.isfinite(value):
            raise ValueError(f"{name} is not finite")
        return value
    return default


def parse_gnss_sample(sample: Any) -> GnssSample:
    """Normalize current and legacy AirSim GpsData layouts."""
    report = getattr(sample, "gnss", sample)
    geo_point = getattr(report, "geo_point", sample)
    latitude = _first_float(geo_point, ("latitude",), float("nan"))
    longitude = _first_float(geo_point, ("longitude",), float("nan"))
    altitude = _first_float(geo_point, ("altitude",), float("nan"))
    if not -90.0 <= latitude <= 90.0:
        raise ValueError("latitude is outside [-90, 90]")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError("longitude is outside [-180, 180]")

    fix_value = getattr(report, "fix_type", 0)
    try:
        fix_type = int(fix_value)
    except (TypeError, ValueError):
        fix_type = 0
    valid = bool(getattr(sample, "is_valid", True)) and fix_type >= 2
    return GnssSample(
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
        valid=valid,
        fix_type=fix_type,
        horizontal_accuracy=_first_float(report, ("eph",), 0.0),
        vertical_accuracy=_first_float(report, ("epv",), 0.0),
    )


def navsat_status(sample: GnssSample, minimum_fix_type: int = 2) -> int:
    """Map AirSim fix information to a ROS NavSatStatus code."""
    if sample.valid and sample.fix_type >= minimum_fix_type:
        return NavSatStatus.STATUS_FIX
    return NavSatStatus.STATUS_NO_FIX


def position_covariance(
    sample: GnssSample,
    horizontal_fallback: float,
    vertical_fallback: float,
) -> tuple[float, float, float]:
    """Return diagonal latitude, longitude and altitude variances."""
    horizontal = sample.horizontal_accuracy
    vertical = sample.vertical_accuracy
    if horizontal <= 0.0:
        horizontal = math.sqrt(max(horizontal_fallback, 0.0))
    if vertical <= 0.0:
        vertical = math.sqrt(max(vertical_fallback, 0.0))
    return (horizontal**2, horizontal**2, vertical**2)
