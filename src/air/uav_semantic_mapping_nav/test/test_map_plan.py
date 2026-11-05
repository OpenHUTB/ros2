# -*- coding: utf-8 -*-
"""语义栅格、代价图与 A* 规划。不需要 ROS、不需要 mujoco。"""
import math

import numpy as np
import pytest

from uav_semantic_mapping_nav.core.labels import BLOCKED, N_CLASS, TRAVERSAL_COST
from uav_semantic_mapping_nav.core.metrics import (collision_count,
                                                  path_uncertainty,
                                                  travelled_through)
from uav_semantic_mapping_nav.core.planner import (PlanConfig, astar,
                                                   geometric_cost, path_length,
                                                   smooth_path)
from uav_semantic_mapping_nav.core.semantic_map import (CameraModel, SemMapConfig,
                                                       SemanticGrid)


def _wall_scene(g: SemanticGrid, cam: CameraModel, dist: float = 6.0,
                height: float = 1.6):
    """正前方一堵墙 + 向远处展开的地面，几何自洽的合成观测。"""
    h, w = cam.height, cam.width
    depth = np.zeros((h, w), np.float32)
    seg = np.zeros((h, w), np.int32)
    for v in range(h):
        ang = math.atan((v - cam.cy) / cam.fy)
        if ang > math.radians(2.0):
            depth[v, :] = height / math.sin(ang)
            seg[v, :] = 1
        elif ang > math.radians(-30.0):
            depth[v, :] = dist
            seg[v, :] = 2
    return g.integrate(np.array([0.0, 0.0, 0.0]), depth, seg, cam)


def test_integrate_marks_wall_occupied():
    g = SemanticGrid(SemMapConfig(res=0.25, size=40.0, stride=2))
    cam = CameraModel(width=64, height=48)
    _wall_scene(g, cam)
    occ = g.probability() > 0.6
    assert occ.any()
    xy = np.stack([g.centre(r, c) for r, c in np.argwhere(occ)])
    assert 4.0 < float(np.median(np.linalg.norm(xy, axis=1))) < 8.0


def test_wall_cells_survive_carving():
    """终点不能被当成空域采样点，否则整面墙的对数几率被压成负值。"""
    g = SemanticGrid(SemMapConfig(res=0.25, size=40.0, stride=2))
    cam = CameraModel(width=64, height=48)
    _wall_scene(g, cam)
    assert (g.probability() > 0.6).sum() > 5
    assert g.logodds.max() > 0.0


def test_semantic_votes_use_labels():
    g = SemanticGrid(SemMapConfig(res=0.25, size=40.0, stride=2))
    cam = CameraModel(width=64, height=48)
    _wall_scene(g, cam)
    seen = set(np.unique(g.semantic()[g.observed()]).tolist())
    assert 1 in seen and 2 in seen


def test_ground_is_free_and_high_is_occupied():
    g = SemanticGrid(SemMapConfig(res=0.25, size=40.0, stride=2))
    cam = CameraModel(width=64, height=48)
    _wall_scene(g, cam)
    obs = g.observed()
    assert g.free[obs].sum() > 0          # 地面标了空域
    assert g.hits[obs].sum() > 0          # 墙标了命中


def test_cost_map_blocks_occupied():
    g = SemanticGrid(SemMapConfig(res=0.25, size=40.0, stride=2))
    cam = CameraModel(width=64, height=48)
    _wall_scene(g, cam)
    c = g.cost_map()
    assert np.isinf(c[g.probability() > 0.7]).all()


def test_geometric_cost_ignores_class():
    g = SemanticGrid(SemMapConfig(res=0.25, size=40.0, stride=2))
    cam = CameraModel(width=64, height=48)
    _wall_scene(g, cam)
    cg = geometric_cost(g)
    cs = g.cost_map()
    # 墙在两张图上都不可通行；但语义图在植被处会额外加价（这里没有植被）
    assert np.isinf(cg[g.probability() > 0.7]).all()
    assert cg.shape == cs.shape


# --------------------------------------------------------------------------- #
def test_astar_straight_line():
    cost = np.ones((50, 50))
    r = astar(cost, (5, 5), (40, 40))
    assert r.ok
    # 容差 1 格，会在 (39,39) 停下：34 步对角
    assert r.cells[0] == (5, 5)
    assert abs(r.cells[-1][0] - 40) <= 1 and abs(r.cells[-1][1] - 40) <= 1
    assert abs(r.cost - 34 * math.sqrt(2)) < 0.5
    # 代价 = 步长 × 格代价（全 1）应等于几何长度
    assert abs(r.cost - math.hypot(r.cells[-1][0] - 5,
                                   r.cells[-1][1] - 5)) < 0.5


def test_astar_goes_through_gap():
    cost = np.ones((60, 60))
    cost[:, 30] = np.inf
    cost[28:32, 30] = 1.0
    r = astar(cost, (5, 5), (5, 50))
    assert r.ok
    rows = {rr for rr, cc in r.cells if cc == 30}
    assert rows and all(28 <= rr <= 31 for rr in rows)


def test_astar_reports_blocked():
    cost = np.ones((40, 40))
    cost[:, 20] = np.inf
    r = astar(cost, (5, 5), (5, 35))
    assert not r.ok and r.reason


def test_astar_avoids_high_cost_band():
    n = 60
    cost = np.ones((n, n))
    cost[20:40, :] = 6.0
    r = astar(cost, (25, 2), (25, 56))
    assert r.ok
    assert r.cost < 0.7 * 54 * 3.5


def test_smooth_path_is_valid_and_shorter():
    cost = np.ones((60, 60))
    r = astar(cost, (2, 2), (55, 50))
    s = smooth_path(cost, r.cells)
    assert len(s) <= len(r.cells)
    assert s[0] == r.cells[0] and s[-1] == r.cells[-1]


def test_smooth_path_stays_free():
    cost = np.ones((60, 60))
    cost[:, 30] = np.inf
    cost[28:32, 30] = 1.0
    r = astar(cost, (5, 5), (5, 50))
    s = smooth_path(cost, r.cells)
    for i in range(len(s) - 1):
        a, b = s[i], s[i + 1]
        ts = np.linspace(0, 1, 60)
        rr = np.rint(a[0] + ts * (b[0] - a[0])).astype(int)
        cc = np.rint(a[1] + ts * (b[1] - a[1])).astype(int)
        assert np.all(np.isfinite(cost[rr, cc]))


def test_path_metrics():
    g = SemanticGrid(SemMapConfig(res=0.5, size=20.0))
    cost = np.ones((g.n, g.n))
    cost[:, g.n // 2] = np.inf
    cost[g.n // 2 - 2:g.n // 2 + 2, g.n // 2] = 1.0
    start = (2, 2)
    goal = (2, g.n - 3)
    r = astar(cost, start, goal)
    assert r.ok
    assert path_length(g, r.cells) > 0
    xy = np.asarray([g.centre(a, b) for a, b in r.cells])
    assert collision_count(cost, xy, g) == 0
    assert travelled_through(cost, xy, g) == pytest.approx(1.0, abs=0.4)


def test_travelled_through_detects_blocked():
    g = SemanticGrid(SemMapConfig(res=0.5, size=20.0))
    cost = np.ones((g.n, g.n))
    cost[5, :] = np.inf
    xy = np.asarray([g.centre(5, c) for c in range(4)])
    assert collision_count(cost, xy, g) == 4
