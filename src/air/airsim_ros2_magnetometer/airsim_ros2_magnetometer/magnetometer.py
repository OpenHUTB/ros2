"""AirSim magnetometer parsing and ROS message conversion."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from builtin_interfaces.msg import Time
from sensor_msgs.msg import MagneticField


@dataclass(frozen=True)
class MagnetometerSample:
    field: tuple[float, float, float]
    covariance: float


def parse_magnetometer_sample(sample: Any):
    value = sample.magnetic_field_body
    field = (float(value.x_val), float(value.y_val), float(value.z_val))
    if not all(math.isfinite(item) for item in field):
        raise ValueError("magnetic field contains a non-finite value")
    covariance = float(getattr(sample, "magnetic_field_covariance", 0.0) or 0.0)
    if not math.isfinite(covariance) or covariance < 0.0:
        raise ValueError("magnetic field covariance must be finite and non-negative")
    if covariance <= 0.0:
        covariance = 0.0
    forward, right, down = field
    return MagnetometerSample((forward, -right, -down), covariance)


def build_message(sample: MagnetometerSample, *, frame_id: str, stamp: Time, field_scale: float):
    message = MagneticField()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.magnetic_field.x = sample.field[0] * field_scale
    message.magnetic_field.y = sample.field[1] * field_scale
    message.magnetic_field.z = sample.field[2] * field_scale
    for index in (0, 4, 8):
        message.magnetic_field_covariance[index] = (
            sample.covariance * field_scale * field_scale
        )
    return message
