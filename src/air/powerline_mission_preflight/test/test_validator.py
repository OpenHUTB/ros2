import pytest

from powerline_mission_preflight.validator import validate_mission


def viewpoint(pole, side, x, y, z=20, yaw=0):
    return {"pole_index": pole, "side": side, "x": x, "y": y, "z": z, "yaw_deg": yaw}


def test_valid_mission_is_ready_and_reports_metrics():
    mission = {
        "route": [
            viewpoint(1, "left", 0, 8, yaw=-90),
            viewpoint(1, "right", 0, -8, yaw=90),
            viewpoint(2, "right", 40, -8, yaw=90),
            viewpoint(2, "left", 40, 8, yaw=-90),
        ]
    }

    report = validate_mission(mission)

    assert report["ready"] is True
    assert report["errors"] == []
    assert report["warnings"] == []
    assert report["metrics"]["viewpoint_count"] == 4
    assert report["metrics"]["pole_count"] == 2
    assert report["metrics"]["distance_m"] == pytest.approx(72.0)


def test_invalid_altitude_and_yaw_block_mission():
    mission = {"route": [viewpoint(1, "left", 0, 0, z=2, yaw=200)]}

    report = validate_mission(mission)

    assert report["ready"] is False
    assert len(report["errors"]) == 2


def test_missing_side_and_long_leg_create_warnings():
    mission = {
        "route": [
            viewpoint(1, "left", 0, 0),
            viewpoint(2, "left", 100, 0),
        ]
    }

    report = validate_mission(mission, max_leg_distance=50)

    assert report["ready"] is True
    assert len(report["warnings"]) == 3


@pytest.mark.parametrize("mission", [{}, {"route": []}, {"route": "invalid"}])
def test_route_must_be_a_non_empty_list(mission):
    report = validate_mission(mission)

    assert report["ready"] is False
    assert report["metrics"]["viewpoint_count"] == 0


def test_missing_fields_and_non_finite_values_are_rejected():
    report = validate_mission(
        {"route": [{"pole_index": 1}, viewpoint(2, "right", float("nan"), 0)]}
    )

    assert report["ready"] is False
    assert len(report["errors"]) == 2
