"""建图与落点选择相关的单元测试（纯 numpy，不需要 mujoco）。"""

import numpy as np
import pytest

from uav_fault_emergency_landing import landing as L
from uav_fault_emergency_landing import mapping as M


def _grid(size=10.0, cell=1.0):
    return M.GridConfig(size_m=size, cell_m=cell)


def test_grid_shape_and_origin():
    cfg = _grid()
    assert cfg.shape == (10, 10)
    assert cfg.origin_xy == pytest.approx([-5.0, -5.0])


def test_world_to_cells_filters_out_of_range():
    cfg = _grid()
    pts = np.array([[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [-20.0, 5.0, 0.0]])
    _, _, valid = M.world_to_cells(cfg, pts)
    assert valid.shape == (3,)
    assert valid.tolist() == [True, False, False]


def test_height_map_keeps_highest_point():
    hm = M.HeightMap(_grid())
    hm.update(np.array([[0.0, 0.0, 0.4], [0.1, 0.1, 1.6]]))
    assert hm.height[5, 5] == pytest.approx(1.6)
    assert int(hm.observed.sum()) == 1


def test_height_map_accepts_tiny_negative_z():
    """深度图反投影的地面点会落在 -1e-8 附近，不能被当成越界丢掉。"""
    hm = M.HeightMap(_grid())
    hm.update(np.array([[0.0, 0.0, -2.86e-08]]))
    assert int(hm.observed.sum()) == 1


def test_obstacle_mask_threshold():
    hm = M.HeightMap(_grid())
    hm.update(np.array([[0.0, 0.0, M.OBSTACLE_HEIGHT + 0.5], [2.0, 2.0, 0.1]]))
    mask = hm.obstacle_mask()
    assert bool(mask[5, 5])
    assert not bool(mask[7, 7])


def test_distance_field_values():
    obstacle = np.zeros((10, 10), dtype=bool)
    obstacle[5, 5] = True
    dist = M._distance_to_obstacle(obstacle, 1.0, radius_cells=8)
    assert dist[5, 5] == pytest.approx(0.0)
    assert dist[5, 6] == pytest.approx(1.0)
    assert dist[5, 7] == pytest.approx(2.0)
    assert dist[5, 9] == pytest.approx(4.0)


def test_rule_score_prefers_flat_observed_ground():
    hm = M.HeightMap(_grid())
    xs = np.arange(-4.0, 4.1, 0.4)
    flat = np.array([[x, y, 0.0] for x in xs for y in xs])
    hm.update(flat)
    score = M.rule_landing_score(hm)
    assert float(score.max()) > 0.2
    assert float(score.min()) == pytest.approx(0.0)
    # 中心已观测且平坦，分数应高于边缘未观测格
    assert score[5, 5] > score[0, 0]


def test_height_map_input_marks_unobserved_as_high():
    hm = M.HeightMap(_grid())
    hm.update(np.array([[0.0, 0.0, 0.0]]))
    grid = M.height_map_input(hm)
    assert grid.shape == (1, 10, 10)
    assert float(grid[0][~hm.observed].min()) >= 1.0 - 1e-6


def test_depth_to_points_plane():
    depth = np.full((16, 16), 3.0)
    pts = M.depth_to_points(depth, np.array([0.0, 0.0, 5.0]), np.eye(3), 60.0)
    assert len(pts) == 16 * 16
    assert pts[:, 2].mean() == pytest.approx(2.0, abs=1e-3)


def test_select_target_penalises_distance():
    hm = M.HeightMap(_grid())
    score = np.zeros(hm.shape)
    score[2, 2] = 0.9
    score[5, 5] = 0.9
    picked = L.select_target(score, hm, position_xy=np.array([0.0, 0.0]),
                            min_score=0.1, max_distance=10.0)
    assert picked is not None
    row, col, value = picked
    assert (row, col) == (5, 5)
    assert value == pytest.approx(0.9)


def test_select_target_returns_none_when_score_too_low():
    hm = M.HeightMap(_grid())
    score = np.full(hm.shape, 0.01)
    picked = L.select_target(score, hm, position_xy=np.array([0.0, 0.0]),
                             min_score=0.1, max_distance=12.0)
    assert picked is None


def test_expert_velocity_stages():
    gains = np.ones(4)
    v = L.expert_velocity(np.array([1.0, 0.0, -3.0]), np.zeros(3), gains, 5.0)
    assert v[0] > 0.0 and v[2] < 0.0
    low = L.expert_velocity(np.array([1.0, 0.0, -3.0]), np.zeros(3), gains, 0.2)
    assert abs(low[2]) < abs(v[2])


def test_expert_velocity_respects_authority():
    # 权限低于 min_authority 时专家律会额外加快下降，尽快落地
    weak = L.expert_velocity(np.array([0.0, 0.0, -5.0]), np.zeros(3),
                             np.array([0.1, 0.1, 0.1, 0.1]), 5.0)
    strong = L.expert_velocity(np.array([0.0, 0.0, -5.0]), np.zeros(3),
                               np.ones(4), 5.0)
    assert abs(weak[2]) > abs(strong[2])


def test_guidance_features_dimension():
    feats = L.guidance_features(np.array([1.0, 2.0, -3.0]), np.zeros(3), np.ones(4), 5.0)
    assert feats.shape == (L.GUIDANCE_DIM,)
    assert np.all(np.isfinite(feats))


def test_touchdown_envelope():
    assert L.touchdown_ok(0.5, 0.4, 10.0, True)
    assert not L.touchdown_ok(3.0, 0.4, 10.0, True)
    assert not L.touchdown_ok(0.5, 3.0, 10.0, True)
    assert not L.touchdown_ok(0.5, 0.4, 60.0, True)


def test_landing_config_defaults_are_physical():
    cfg = L.LandingConfig()
    assert cfg.touch_z > 0.0
    assert cfg.max_descent > cfg.touch_descent
    assert 0.0 < cfg.min_authority < 1.0
