import math

import pytest
from fire_detection_drone.explorer_core import (
    DetectionInfo,
    ExplorerConfig,
    ExplorerCore,
    ExplorerState,
    Pose2D,
)


def make_core(**overrides) -> ExplorerCore:
    defaults = dict(
        pattern='lawnmower',
        search_width=10.0,
        search_height=6.0,
        lane_spacing=2.0,
        search_altitude=3.0,
        confirm_hover_sec=2.0,
        alert_grace_sec=1.0,
        evidence_sec=3.0,
        max_mission_sec=100.0,
        return_home_tolerance=0.5,
        land_after_return=True,
    )
    defaults.update(overrides)
    return ExplorerCore(ExplorerConfig(**defaults))


def test_lawnmower_covers_search_area():
    core = make_core()
    points = core.waypoints
    assert len(points) == 8
    assert min(point.x for point in points) == pytest.approx(-5.0)
    assert max(point.x for point in points) == pytest.approx(5.0)
    assert min(point.y for point in points) == pytest.approx(-3.0)
    assert max(point.y for point in points) == pytest.approx(3.0)
    assert all(point.z == pytest.approx(3.0) for point in points)


def test_perimeter_and_spiral_generate_waypoints():
    perimeter = make_core(pattern='perimeter')
    spiral = make_core(pattern='spiral')
    assert len(perimeter.waypoints) == 4
    assert len(spiral.waypoints) > 10
    assert all(abs(point.x) <= 5.0 for point in perimeter.waypoints)
    assert all(abs(point.y) <= 3.0 for point in perimeter.waypoints)


def test_fire_mission_state_sequence():
    core = make_core()
    pose = Pose2D(x=0.0, y=0.0, z=3.0, yaw=0.0)

    command = core.update(now=0.0, pose=pose, alert=False)
    assert command.state == ExplorerState.PATROL

    command = core.update(now=1.0, pose=pose, alert=True)
    assert command.state == ExplorerState.CONFIRM
    assert command.linear_x == pytest.approx(0.0)
    assert command.linear_y == pytest.approx(0.0)

    command = core.update(now=2.0, pose=pose, alert=True)
    assert command.state == ExplorerState.CONFIRM

    command = core.update(now=3.1, pose=pose, alert=True)
    assert command.state == ExplorerState.EVIDENCE

    away = Pose2D(x=4.0, y=0.0, z=3.0, yaw=math.pi)
    command = core.update(now=6.2, pose=away, alert=True)
    assert command.state == ExplorerState.RETURN_HOME

    command = core.update(now=7.0, pose=away, alert=True)
    assert command.state == ExplorerState.RETURN_HOME
    assert command.linear_x > 0.0

    home = Pose2D(x=0.0, y=0.0, z=3.0, yaw=math.pi)
    command = core.update(now=8.0, pose=home, alert=True)
    assert command.state == ExplorerState.LAND
    assert command.land is True

    command = core.update(now=9.0, pose=home, alert=True)
    assert command.state == ExplorerState.LAND
    assert command.land is False


def test_false_alarm_returns_to_patrol():
    core = make_core()
    pose = Pose2D(x=0.0, y=0.0, z=3.0)
    core.update(now=0.0, pose=pose, alert=True)
    core.update(now=0.3, pose=pose, alert=False)
    command = core.update(now=0.9, pose=pose, alert=False)
    assert command.state == ExplorerState.CONFIRM
    command = core.update(now=1.5, pose=pose, alert=False)
    assert command.state == ExplorerState.PATROL


def test_detection_centres_camera_during_confirmation():
    core = make_core()
    pose = Pose2D(x=0.0, y=0.0, z=3.0)
    core.update(now=0.0, pose=pose, alert=True)
    detection = DetectionInfo(center_x=0.85, center_y=0.5, visible=True)
    command = core.update(now=0.2, pose=pose, alert=True, detection=detection)
    assert command.angular_z < 0.0


def test_abort_returns_home():
    core = make_core()
    pose = Pose2D(x=2.0, y=1.0, z=3.0, yaw=0.0)
    core.update(now=0.0, pose=pose, alert=False)
    command = core.update(now=1.0, pose=pose, alert=False, abort=True)
    assert command.state == ExplorerState.RETURN_HOME


def test_mission_timeout_returns_home():
    core = make_core(max_mission_sec=5.0)
    pose = Pose2D(x=2.0, y=1.0, z=3.0)
    core.update(now=0.0, pose=pose, alert=False)
    command = core.update(now=5.1, pose=pose, alert=False)
    assert command.state == ExplorerState.RETURN_HOME
