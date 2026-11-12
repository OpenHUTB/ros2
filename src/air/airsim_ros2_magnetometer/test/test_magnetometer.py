"""Unit tests for magnetometer parsing and message conversion."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from airsim_ros2_magnetometer.magnetometer import (
    build_message,
    parse_magnetometer_sample,
)


def test_frd_to_flu_and_scale() -> None:
    sample = parse_magnetometer_sample(
        SimpleNamespace(
            magnetic_field_body=SimpleNamespace(x_val=1.0, y_val=2.0, z_val=3.0),
            magnetic_field_covariance=4.0,
        ),
    )
    message = build_message(
        sample,
        frame_id="mag_link",
        stamp=Time(sec=2, nanosec=3),
        field_scale=1e-4,
    )
    assert sample.field == (1.0, -2.0, -3.0)
    assert message.header.frame_id == "mag_link"
    assert message.magnetic_field.x == pytest.approx(1e-4)
    assert message.magnetic_field.y == pytest.approx(-2e-4)
    assert message.magnetic_field.z == pytest.approx(-3e-4)
    for index in (0, 4, 8):
        assert message.magnetic_field_covariance[index] == pytest.approx(4e-8)


def test_unknown_covariance_stays_zero() -> None:
    sample = parse_magnetometer_sample(
        SimpleNamespace(
            magnetic_field_body=SimpleNamespace(x_val=1.0, y_val=2.0, z_val=3.0),
            magnetic_field_covariance=0.0,
        )
    )
    message = build_message(
        sample,
        frame_id="mag_link",
        stamp=Time(sec=0, nanosec=0),
        field_scale=1.0,
    )
    assert list(message.magnetic_field_covariance) == [0.0] * 9


def test_non_finite_field_is_rejected() -> None:
    with pytest.raises(ValueError, match="non-finite"):
        parse_magnetometer_sample(
            SimpleNamespace(
                magnetic_field_body=SimpleNamespace(
                    x_val=math.nan, y_val=0.0, z_val=0.0
                )
            )
        )
