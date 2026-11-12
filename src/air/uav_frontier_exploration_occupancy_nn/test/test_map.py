"""占据栅格地图与射线探测测试。"""

import math

from uav_frontier_exploration_occupancy_nn.occupancy_map import (
    OccupancyMap, FREE, OCCUPIED, UNKNOWN)


def test_raycast_marks_free_cells():
    m = OccupancyMap()
    hit = m.raycast(0.5, 0.5, 0.0, 2.0)
    assert hit is False
    free = m.explored_free_count()
    assert free > 5
    for r in range(m.h):
        for c in range(m.w):
            assert m.grid[r, c] in (FREE, UNKNOWN)


def test_raycast_hits_obstacle():
    m = OccupancyMap()
    m.add_obstacle_rect(2.0, 1.0, 3.0, 2.0)
    hit = m.raycast(0.5, 1.5, 0.0, 3.0)
    assert hit is True
    c, r = m.world_to_cell(2.1, 1.5)
    assert m.grid[r, c] == OCCUPIED


def test_frontier_cells_boundary():
    m = OccupancyMap()
    m.raycast(0.5, 0.5, 0.0, 1.5)
    frontiers = m.frontier_cells()
    assert len(frontiers) > 0
    for c, r in frontiers:
        assert m.grid[r, c] == FREE
