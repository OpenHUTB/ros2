"""core.py 的 pytest 单元测试：不依赖 ROS。"""

import math

import pytest

from uav_los_waypoint_guidance import core


def test_wrap_angle():
    assert core.wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert core.wrap_angle(-0.5) == pytest.approx(-0.5)


def test_on_line_has_no_cross_error():
    along, cross = core.segment_tracking((0.0, 0.0), (10.0, 0.0), (5.0, 0.0))
    assert along == pytest.approx(5.0)
    assert cross == pytest.approx(0.0)


def test_cross_error_sign():
    _, cross = core.segment_tracking((0.0, 0.0), (10.0, 0.0), (5.0, 1.0))
    assert cross == pytest.approx(1.0)


def test_heading_equals_path_on_line():
    heading = core.los_heading((0.0, 0.0), (10.0, 0.0), (5.0, 0.0))
    assert heading == pytest.approx(0.0)


def test_offset_corrects_towards_path():
    heading = core.los_heading((0.0, 0.0), (10.0, 0.0), (5.0, 1.0), lookahead=2.0)
    assert heading < 0.0


def test_body_velocity_straight():
    vx, vy = core.body_velocity(0.0, 2.0)
    assert vx == pytest.approx(2.0)
    assert vy == pytest.approx(0.0)


def test_body_velocity_turn():
    vx, vy = core.body_velocity(0.5, 2.0)
    assert vy > 0.0
    assert vx < 2.0


def test_waypoint_reached():
    assert core.waypoint_reached((0.1, 0.1), (0.0, 0.0), 0.5)
    assert not core.waypoint_reached((2.0, 0.0), (0.0, 0.0), 0.5)


def test_guidance_step_returns_distance():
    vx, vy, err, dist = core.guidance_step((0.0, 0.0), 0.0, (0.0, 0.0), (5.0, 0.0))
    assert dist == pytest.approx(5.0)
    assert err == pytest.approx(0.0)
    assert vx > 0.0
