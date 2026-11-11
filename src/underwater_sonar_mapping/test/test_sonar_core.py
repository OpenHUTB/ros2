import math

import numpy as np
import pytest

from underwater_sonar_mapping.sonar_core import (
    DEFAULT_INITIAL_POSE,
    DEFAULT_MAP_ORIGIN,
    DEFAULT_OBSTACLES,
    integrate_scan,
    raycast_scan,
    validate_obstacles,
)


def test_raycast_returns_nearest_circle_surface_distance():
    scan = raycast_scan(0.0, 0.0, 0.0, [(4.0, 0.0, 1.0), (2.0, 0.0, 0.25)], beam_count=1)

    assert scan == ((0.0, 1.75, True),)


def test_raycast_reports_no_return_at_max_range():
    scan = raycast_scan(0.0, 0.0, 0.0, [(0.0, 4.0, 0.5)], beam_count=3, max_range=6.0)

    assert all(distance == 6.0 and not hit for _, distance, hit in scan)


def test_raycast_rotates_beam_angles_with_pose():
    scan = raycast_scan(0.0, 0.0, math.pi / 2, [], beam_count=1)

    assert scan[0][0] == pytest.approx(math.pi / 2)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"beam_count": 0},
        {"field_of_view": 0.0},
        {"max_range": float("inf")},
    ],
)
def test_raycast_rejects_invalid_scan_configuration(kwargs):
    with pytest.raises(ValueError):
        raycast_scan(0.0, 0.0, 0.0, [], **kwargs)


def test_obstacles_require_finite_positive_radius_triples():
    with pytest.raises(ValueError):
        validate_obstacles([(1.0, 2.0, 0.0)])
    with pytest.raises(ValueError):
        validate_obstacles([(1.0, float("nan"), 1.0)])


def test_scan_marks_free_cells_and_hit_endpoint():
    grid = np.full((10, 10), -1, dtype=np.int8)

    updated = integrate_scan(grid, (0.0, 0.0), 1.0, 1.0, 1.5, [(0.0, 4.0, True)])

    assert updated[1, 1] == 0
    assert updated[1, 5] == 100
    assert updated[8, 8] == -1


def test_hit_outside_map_does_not_turn_map_edge_into_obstacle():
    grid = np.full((4, 4), -1, dtype=np.int8)

    updated = integrate_scan(grid, (0.0, 0.0), 1.0, 1.0, 1.5, [(0.0, 8.0, True)])

    assert np.all(updated[:, 3] != 100)


def test_default_scene_produces_free_cells_and_echoes_inside_map():
    grid = np.full((80, 80), -1, dtype=np.int8)
    x, y, yaw = DEFAULT_INITIAL_POSE
    scan = raycast_scan(x, y, yaw, DEFAULT_OBSTACLES, beam_count=90, max_range=10.0)

    updated = integrate_scan(grid, DEFAULT_MAP_ORIGIN, 0.25, x, y, scan)

    assert np.count_nonzero(updated == 0) > 0
    assert np.count_nonzero(updated == 100) > 0

