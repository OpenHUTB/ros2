"""star.py 的纯函数测试（不需要 ROS / turtlesim）。"""

from __future__ import annotations

import math

import pytest

from turtle_star.star import (
    EDGES,
    TOTAL_TURN,
    TURN_ANGLE,
    closure_error,
    edge_duration,
    palette,
    turn_duration,
    vertices,
)


def test_turn_angle_is_144_degrees():
    assert TURN_ANGLE == pytest.approx(math.radians(144.0))
    assert TOTAL_TURN == pytest.approx(4 * math.pi)
    assert EDGES == 5


def test_polygon_closes():
    for side in (0.5, 1.0, 2.0):
        assert closure_error(side) < 1e-9


def test_vertices_count():
    points = vertices(1.0)
    assert len(points) == EDGES + 1          # 5 条边 = 6 个点（最后回到起点）
    assert points[0] == pytest.approx((0.0, 0.0))


def test_first_edge_is_along_x():
    points = vertices(2.0)
    assert points[1] == pytest.approx((2.0, 0.0))


def test_edge_lengths_equal():
    points = vertices(1.5)
    for a, b in zip(points, points[1:]):
        assert math.hypot(b[0] - a[0], b[1] - a[1]) == pytest.approx(1.5)


def test_vertices_bounded():
    points = vertices(1.0)
    for x, y in points:
        assert abs(x) <= 2.0 and abs(y) <= 2.0


def test_durations():
    assert edge_duration(2.0, 1.0) == pytest.approx(2.0)
    assert edge_duration(1.0, 2.0) == pytest.approx(0.5)
    assert turn_duration(1.0) == pytest.approx(math.radians(144.0))


@pytest.mark.parametrize('bad', [0.0, -1.0])
def test_bad_inputs_rejected(bad):
    with pytest.raises(ValueError):
        vertices(bad)
    with pytest.raises(ValueError):
        edge_duration(1.0, bad)
    with pytest.raises(ValueError):
        turn_duration(bad)


def test_palette():
    colors = palette()
    assert len(colors) == EDGES
    assert all(len(c) == 3 for c in colors)
    assert len(set(colors)) == EDGES        # 五种不同颜色
    assert len(palette(2)) == 2
