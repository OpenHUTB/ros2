"""Unit tests for AirSim barometer normalization."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from airsim_ros2_barometer.barometer import parse_barometer_sample


def test_parse_valid_sample_converts_qnh_hpa_to_pa() -> None:
    sample = parse_barometer_sample(
        SimpleNamespace(
            altitude=12.5,
            pressure=101250.0,
            qnh=1013.25,
        )
    )
    assert sample.altitude == pytest.approx(12.5)
    assert sample.pressure == pytest.approx(101250.0)
    assert sample.qnh_pa == pytest.approx(101325.0)


@pytest.mark.parametrize("pressure", [0.0, -1.0, float("nan")])
def test_invalid_pressure_is_rejected(pressure: float) -> None:
    sample = SimpleNamespace(
        altitude=1.0,
        pressure=pressure,
        qnh=101325.0,
    )
    with pytest.raises(ValueError, match="pressure"):
        parse_barometer_sample(sample)


@pytest.mark.parametrize("altitude", [float("inf"), float("-inf")])
def test_invalid_altitude_is_rejected(altitude: float) -> None:
    sample = SimpleNamespace(
        altitude=altitude,
        pressure=101325.0,
        qnh=101325.0,
    )
    with pytest.raises(ValueError, match="altitude"):
        parse_barometer_sample(sample)


@pytest.mark.parametrize(
    "qnh",
    [0.0, -1.0, float("nan"), float("inf"), float("-inf")],
)
def test_invalid_qnh_is_rejected(qnh: float) -> None:
    sample = SimpleNamespace(
        altitude=1.0,
        pressure=101325.0,
        qnh=qnh,
    )
    with pytest.raises(ValueError, match="QNH"):
        parse_barometer_sample(sample)
