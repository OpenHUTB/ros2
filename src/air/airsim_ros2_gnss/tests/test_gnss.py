"""Unit tests for AirSim GNSS normalization."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from sensor_msgs.msg import NavSatStatus

from airsim_ros2_gnss.gnss import (
    navsat_status,
    parse_gnss_sample,
    position_covariance,
)


def test_parse_current_airsim_gps_layout() -> None:
    sample = SimpleNamespace(
        is_valid=True,
        gnss=SimpleNamespace(
            geo_point=SimpleNamespace(
                latitude=31.23,
                longitude=121.47,
                altitude=12.5,
            ),
            eph=1.2,
            epv=2.4,
            fix_type=3,
        ),
    )
    parsed = parse_gnss_sample(sample)
    assert parsed.latitude == pytest.approx(31.23)
    assert parsed.longitude == pytest.approx(121.47)
    assert parsed.altitude == pytest.approx(12.5)
    assert parsed.fix_type == 3
    assert navsat_status(parsed) == NavSatStatus.STATUS_FIX


def test_parse_legacy_gps_layout() -> None:
    sample = SimpleNamespace(
        latitude=31.2,
        longitude=121.4,
        altitude=10.0,
        eph=0.0,
        epv=0.0,
        fix_type=2,
    )
    parsed = parse_gnss_sample(sample)
    assert parsed.valid
    assert parsed.horizontal_accuracy == pytest.approx(0.0)


def test_invalid_fix_maps_to_no_fix() -> None:
    sample = SimpleNamespace(
        is_valid=False,
        gnss=SimpleNamespace(
            geo_point=SimpleNamespace(
                latitude=31.2,
                longitude=121.4,
                altitude=10.0,
            ),
            eph=1.0,
            epv=2.0,
            fix_type=3,
        ),
    )
    parsed = parse_gnss_sample(sample)
    assert navsat_status(parsed) == NavSatStatus.STATUS_NO_FIX


def test_covariance_uses_accuracy_or_fallback() -> None:
    accurate = SimpleNamespace(
        is_valid=True,
        gnss=SimpleNamespace(
            geo_point=SimpleNamespace(
                latitude=31.2,
                longitude=121.4,
                altitude=10.0,
            ),
            eph=2.0,
            epv=3.0,
            fix_type=3,
        ),
    )
    fallback = SimpleNamespace(
        is_valid=True,
        gnss=SimpleNamespace(
            geo_point=SimpleNamespace(
                latitude=31.2,
                longitude=121.4,
                altitude=10.0,
            ),
            eph=0.0,
            epv=0.0,
            fix_type=3,
        ),
    )
    assert position_covariance(parse_gnss_sample(accurate), 25.0, 49.0) == (
        4.0,
        4.0,
        9.0,
    )
    assert position_covariance(parse_gnss_sample(fallback), 25.0, 49.0) == (
        25.0,
        25.0,
        49.0,
    )


def test_invalid_latitude_is_rejected() -> None:
    sample = SimpleNamespace(
        is_valid=True,
        gnss=SimpleNamespace(
            geo_point=SimpleNamespace(
                latitude=91.0,
                longitude=121.4,
                altitude=10.0,
            ),
            fix_type=3,
        ),
    )
    with pytest.raises(ValueError, match="latitude"):
        parse_gnss_sample(sample)
