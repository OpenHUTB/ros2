#!/usr/bin/env python3
"""单元测试：覆盖栅格地图、A* 搜索、路径后处理、时间参数化与安全辅助。

直接运行即可（不依赖 pytest）：

    python3 tests/test_core.py
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ros2_astar_planner.astar import (  # noqa: E402
    AStarConfig,
    AStarPlanner,
    count_turns,
    path_length,
    path_world,
)
from ros2_astar_planner.gridmap import (  # noqa: E402
    GridMap,
    GridSpec,
    corridor_map,
    gate_map,
    maze_map,
    random_obstacle_map,
)
from ros2_astar_planner.safety import (  # noqa: E402
    DeadlineGuard,
    choose_connected_endpoints,
    inflation_cells,
    nearest_free_cell,
    pick_sturdy_endpoints,
    resolve_endpoint,
    sanitize_positive,
    validate_map,
)
from ros2_astar_planner.smoothing import (  # noqa: E402
    count_turns_world,
    gradient_smooth,
    polyline_length,
    segment_is_free,
    shortcut,
    smooth_path,
)
from ros2_astar_planner.trajectory import (  # noqa: E402
    check_acceleration,
    compute_curvature,
    sample_trajectory,
    time_parameterize,
)

TESTS = []


def test(fn):
    TESTS.append(fn)
    return fn


# 这是给独立运行入口注册用例的装饰器，不是测试函数本身。
# 名字以 test 开头会被 pytest 收集，并因缺少 fn 参数报 fixture 错误。
test.__test__ = False


# ============================ 栅格地图 ============================


@test
def test_gridspec_rejects_invalid():
    for bad in (dict(width=0, height=5, resolution=0.1), dict(width=5, height=5, resolution=0.0)):
        try:
            GridSpec(**bad)
            raise AssertionError("非法 GridSpec 未抛异常")
        except ValueError:
            pass


@test
def test_gridmap_shape_mismatch():
    try:
        GridMap(np.zeros((3, 3), dtype=int), GridSpec(5, 5, 0.1))
        raise AssertionError("形状不匹配未抛异常")
    except ValueError:
        pass


@test
def test_world_grid_roundtrip():
    gm = GridMap.empty(20, 10, 0.05)
    i, j = gm.to_grid(0.375, 0.125)
    x, y = gm.to_world(i, j)
    assert abs(x - 0.375) < 0.05 and abs(y - 0.125) < 0.05


@test
def test_out_of_bounds_is_not_free():
    gm = GridMap.empty(10, 10, 0.1)
    assert not gm.is_free(-1, 0)
    assert not gm.is_free(0, 10)
    assert not gm.is_free(99, 99)


@test
def test_set_obstacle_and_add_rect():
    gm = GridMap.empty(10, 10, 0.1)
    gm.set_obstacle(2, 3)
    assert gm.grid[2, 3] == 1
    gm.add_rect(5, 5, 7, 7)
    assert gm.grid[5:8, 5:8].sum() == 9


@test
def test_inflate_grows_obstacles():
    gm = GridMap.empty(21, 21, 0.1)
    gm.set_obstacle(10, 10)
    before = int(gm.grid.sum())
    grown = gm.inflate(2)
    after = int(grown.grid.sum())
    assert after > before
    # 膨胀后的形状不变
    assert grown.shape == gm.shape


@test
def test_inflate_zero_returns_copy():
    gm = GridMap.empty(10, 10, 0.1)
    gm.set_obstacle(1, 1)
    grown = gm.inflate(0)
    assert grown.grid.sum() == gm.grid.sum()
    grown.set_obstacle(5, 5)
    assert gm.grid[5, 5] == 0  # 原图未被修改


@test
def test_inflate_blocks_narrow_gap():
    # 一格宽的门洞，膨胀 2 格后必然被堵死
    gm = gate_map(gap_rows=2)
    grown = gm.inflate(2)
    labels, n = grown.connected_components()
    assert n >= 2, "门洞被堵死后应分裂为多个连通域"


@test
def test_free_cells_count():
    gm = GridMap.empty(10, 10, 0.1)
    gm.set_obstacle(0, 0)
    assert len(gm.free_cells()) == 99


@test
def test_connected_components():
    gm = GridMap.empty(10, 10, 0.1)
    gm.add_rect(4, 0, 5, 9)  # 横贯的墙
    labels, n = gm.connected_components()
    assert n == 2
    assert labels[0, 0] != labels[9, 0]


@test
def test_same_component():
    gm = GridMap.empty(10, 10, 0.1)
    gm.add_rect(4, 0, 5, 9)
    assert not gm.same_component((0, 0), (9, 0))
    assert gm.same_component((0, 0), (0, 9))


@test
def test_distance_to_obstacle():
    gm = GridMap.empty(10, 10, 0.1)
    gm.set_obstacle(5, 5)
    d = gm.distance_to_obstacle()
    assert d[5, 5] == 0
    assert d[5, 6] == 1
    assert d[4, 4] == 1
    assert d[0, 0] == 5


@test
def test_builtin_maps_are_valid():
    for gm in (corridor_map(), maze_map(), gate_map(), random_obstacle_map(40, 30, 0.1, 0.1, 1)):
        assert validate_map(gm) is None
        assert len(gm.free_cells()) > 0


@test
def test_random_map_is_reproducible():
    a = random_obstacle_map(30, 30, 0.1, 0.2, seed=5)
    b = random_obstacle_map(30, 30, 0.1, 0.2, seed=5)
    assert np.array_equal(a.grid, b.grid)


# ============================ A* 搜索 ============================


def _simple_scene():
    """一块带一堵中间墙的 21x21 空地。"""
    gm = GridMap.empty(21, 21, 0.1)
    gm.add_rect(0, 0, 20, 20)
    gm.grid[1:20, 1:20] = 0
    gm.add_rect(1, 10, 19, 10)
    gm.grid[9:13, 10] = 0  # 门洞
    return gm


@test
def test_plan_simple_scene_succeeds():
    gm = _simple_scene()
    res = AStarPlanner().plan(gm, (3, 3), (17, 17))
    assert res.success, res.reason
    assert res.n_cells > 0
    assert res.expansions > 0


@test
def test_plan_start_equals_goal():
    gm = GridMap.empty(10, 10, 0.1)
    res = AStarPlanner().plan(gm, (3, 3), (3, 3))
    assert res.success and res.n_cells == 1 and res.cost == 0.0


@test
def test_plan_start_inside_obstacle_fails():
    gm = GridMap.empty(10, 10, 0.1)
    gm.set_obstacle(3, 3)
    res = AStarPlanner().plan(gm, (3, 3), (8, 8))
    assert not res.success and "障碍" in res.reason


@test
def test_plan_goal_out_of_bounds_fails():
    gm = GridMap.empty(10, 10, 0.1)
    res = AStarPlanner().plan(gm, (2, 2), (99, 99))
    assert not res.success and "越界" in res.reason


@test
def test_plan_disconnected_fails():
    gm = GridMap.empty(21, 21, 0.1)
    gm.add_rect(0, 0, 20, 20)
    gm.grid[1:20, 1:20] = 0
    gm.add_rect(1, 10, 19, 10)  # 无门洞
    res = AStarPlanner().plan(gm, (3, 3), (17, 17))
    assert not res.success and "不连通" in res.reason


@test
def test_astar_matches_dijkstra_cost():
    """可采纳启发式必须给出与 Dijkstra 相同的代价（最优性验证）。"""
    gm = maze_map()
    s, g, _ = choose_connected_endpoints(gm, (3, 3), (37, 58))
    a = AStarPlanner(AStarConfig(heuristic="octile")).plan(gm, s, g)
    d = AStarPlanner(AStarConfig(heuristic="zero")).plan(gm, s, g)
    assert a.success and d.success
    assert abs(a.cost - d.cost) < 1e-9, f"{a.cost} vs {d.cost}"


@test
def test_astar_expands_fewer_nodes_than_dijkstra():
    gm = maze_map()
    s, g, _ = choose_connected_endpoints(gm, (3, 3), (37, 58))
    a = AStarPlanner(AStarConfig(heuristic="octile")).plan(gm, s, g)
    d = AStarPlanner(AStarConfig(heuristic="zero")).plan(gm, s, g)
    assert a.expansions < d.expansions


@test
def test_diagonal_beats_four_connected():
    gm = _simple_scene()
    s, g = (3, 3), (17, 17)
    a8 = AStarPlanner(AStarConfig(allow_diagonal=True)).plan(gm, s, g)
    a4 = AStarPlanner(AStarConfig(allow_diagonal=False)).plan(gm, s, g)
    assert a8.success and a4.success
    assert a8.cost < a4.cost


@test
def test_weighted_astar_trades_quality_for_speed():
    gm = maze_map()
    s, g, _ = choose_connected_endpoints(gm, (3, 3), (37, 58))
    opt = AStarPlanner(AStarConfig(weight=1.0)).plan(gm, s, g)
    w = AStarPlanner(AStarConfig(weight=2.0)).plan(gm, s, g)
    assert opt.success and w.success
    assert w.expansions <= opt.expansions
    assert w.cost >= opt.cost - 1e-9


@test
def test_illegal_weight_rejected():
    try:
        AStarConfig(weight=0.5)
        raise AssertionError("weight<1 未被拒绝")
    except ValueError:
        pass


@test
def test_max_expansions_guard():
    gm = maze_map()
    planner = AStarPlanner(AStarConfig(max_expansions=5))
    s, g, _ = choose_connected_endpoints(gm, (3, 3), (37, 58))
    res = planner.plan(gm, s, g)
    assert not res.success and "扩展上限" in res.reason


@test
def test_no_corner_cutting():
    """严格穿角：路径中的每个对角步，两侧至少一格必须是自由格。"""
    gm = _simple_scene()
    res = AStarPlanner().plan(gm, (3, 3), (17, 17))
    assert res.success
    for (i0, j0), (i1, j1) in zip(res.cells[:-1], res.cells[1:]):
        di, dj = i1 - i0, j1 - j0
        if di != 0 and dj != 0:
            # 严格版规则：两侧都不能是障碍
            assert gm.grid[i0, j1] == 0 and gm.grid[i1, j0] == 0


@test
def test_path_length_and_turns():
    cells = [(0, 0), (0, 1), (0, 2), (1, 2)]
    assert abs(path_length(cells) - 3.0) < 1e-9
    assert count_turns(cells) == 1
    assert count_turns([(0, 0), (0, 1)]) == 0


@test
def test_path_world_conversion():
    gm = GridMap.empty(10, 10, 0.1)
    pts = path_world(gm, [(0, 0), (0, 1)])
    assert pts.shape == (2, 2)
    assert abs(pts[1, 0] - pts[0, 0] - 0.1) < 1e-9


# ============================ 路径后处理 ============================


@test
def test_segment_is_free_detects_obstacle():
    gm = GridMap.empty(21, 21, 0.1)
    gm.add_rect(10, 0, 10, 20)
    # 墙在第 10 行，线段要穿过 y ∈ [1.0, 1.1) 才算穿越
    a = np.array([0.55, 1.05])
    b = np.array([2.05, 1.05])
    assert not segment_is_free(gm, a, b)
    # 沿墙一侧但不被穿过的线段
    c = np.array([0.55, 2.05])
    d = np.array([2.05, 2.05])  # 第 20 行，避开墙
    assert segment_is_free(gm, c, d)


@test
def test_shortcut_keeps_path_short_and_safe():
    gm = _simple_scene()
    res = AStarPlanner().plan(gm, (3, 3), (17, 17))
    world = path_world(gm, res.cells)
    cut = shortcut(gm, world, rng=np.random.default_rng(0))
    assert len(cut) <= len(world)
    assert polyline_length(cut) <= polyline_length(world) + 1e-9
    for i in range(len(cut) - 1):
        assert segment_is_free(gm, cut[i], cut[i + 1])


@test
def test_gradient_smooth_keeps_endpoints():
    gm = _simple_scene()
    world = np.array([[0.5, 0.5], [1.0, 1.2], [1.6, 1.4], [2.2, 2.0]])
    out = gradient_smooth(gm, world, iterations=20)
    assert np.allclose(out[0], world[0])
    assert np.allclose(out[-1], world[-1])
    assert np.allclose(out, world, atol=0.5)


@test
def test_smooth_path_reports_safe_metrics():
    gm = _simple_scene()
    res = AStarPlanner().plan(gm, (3, 3), (17, 17))
    world = path_world(gm, res.cells)
    out, metrics = smooth_path(gm, world, seed=1)
    assert metrics["safe"] is True
    assert metrics["final_length"] <= metrics["raw_length"] + 1e-9
    assert metrics["final_points"] <= metrics["raw_points"]
    assert out[0][0] == world[0][0] and out[-1][0] == world[-1][0]


@test
def test_count_turns_world():
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    assert count_turns_world(pts) == 1
    assert count_turns_world(np.array([[0.0, 0.0], [1.0, 0.0]])) == 0


# ============================ 时间参数化 ============================


@test
def test_curvature_of_straight_line_is_zero():
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    kappa = compute_curvature(pts)
    assert np.allclose(kappa, 0.0)


@test
def test_curvature_of_arc_is_positive():
    th = np.linspace(0, math.pi / 2, 12)
    pts = np.stack([np.cos(th), np.sin(th)], axis=1)
    kappa = compute_curvature(pts)
    assert np.max(kappa) > 0.5


@test
def test_time_parameterize_respects_limits():
    pts = np.stack([np.linspace(0, 5, 20), np.zeros(20)], axis=1)
    traj = time_parameterize(pts, v_max=0.6, a_max=0.5)
    assert traj.v_max_actual <= 0.6 + 1e-9
    assert check_acceleration(traj, 0.5)
    assert traj.total_time > 0 and traj.total_length > 0


@test
def test_time_parameterize_starts_and_stops():
    pts = np.stack([np.linspace(0, 3, 15), np.zeros(15)], axis=1)
    traj = time_parameterize(pts, v_max=0.6, a_max=0.5, v_start=0.0, v_end=0.0)
    assert traj.v[0] == 0.0 and traj.v[-1] == 0.0


@test
def test_curvature_slows_the_robot():
    th = np.linspace(0, math.pi, 40)
    pts = np.stack([np.cos(th) * 0.5, np.sin(th) * 0.5], axis=1)
    traj = time_parameterize(pts, v_max=1.0, a_max=1.0, a_lat_max=0.5)
    assert traj.v_max_actual < 1.0


@test
def test_sample_trajectory_shape():
    pts = np.stack([np.linspace(0, 4, 20), np.zeros(20)], axis=1)
    traj = time_parameterize(pts)
    samples = sample_trajectory(traj, dt=0.1)
    assert samples.shape[1] == 3
    assert samples[0, 2] == 0.0
    assert samples[-1, 2] <= traj.total_time + 1e-6


@test
def test_empty_and_single_point_trajectory():
    traj = time_parameterize(np.zeros((0, 2)))
    assert traj.total_time == 0.0
    traj1 = time_parameterize(np.array([[0.0, 0.0]]))
    assert traj1.total_length == 0.0


# ============================ 安全辅助 ============================


@test
def test_sanitize_positive_fallbacks():
    v, warn = sanitize_positive(float("nan"), 0.8, "v")
    assert v == 0.8 and warn
    v, warn = sanitize_positive(-1.0, 0.8, "v")
    assert v == 0.8 and warn
    v, warn = sanitize_positive(0.5, 0.8, "v")
    assert v == 0.5 and warn is None


@test
def test_sanitize_positive_clamps():
    v, warn = sanitize_positive(100.0, 0.8, "v", hi=3.0)
    assert v == 3.0 and warn
    v, warn = sanitize_positive(0.0001, 0.8, "v", lo=0.1)
    assert v == 0.1 and warn


@test
def test_nearest_free_cell():
    gm = GridMap.empty(10, 10, 0.1)
    gm.add_rect(4, 4, 5, 5)
    cell = nearest_free_cell(gm, (4, 4))
    assert cell is not None and gm.is_free(*cell)


@test
def test_resolve_endpoint_fixes_obstacle_goal():
    gm = GridMap.empty(10, 10, 0.1)
    gm.set_obstacle(5, 5)
    fix = resolve_endpoint(gm, (5, 5), "终点")
    assert fix.changed and gm.is_free(*fix.cell)


@test
def test_resolve_endpoint_keeps_valid_goal():
    gm = GridMap.empty(10, 10, 0.1)
    fix = resolve_endpoint(gm, (2, 2), "终点")
    assert not fix.changed and fix.cell == (2, 2)


@test
def test_choose_connected_endpoints_returns_connected_pair():
    gm = maze_map()
    s, g, notes = choose_connected_endpoints(gm, (3, 3), (37, 58))
    assert gm.same_component(s, g)
    assert isinstance(notes, list)


@test
def test_pick_sturdy_endpoints_same_component():
    gm = corridor_map()
    s, g = pick_sturdy_endpoints(gm, min_clearance=2)
    assert gm.same_component(s, g)


@test
def test_validate_map_detects_full_occupancy():
    gm = GridMap(np.ones((5, 5)), GridSpec(5, 5, 0.1))
    assert validate_map(gm) is not None


@test
def test_inflation_cells():
    assert inflation_cells(0.20, 0.1) == 2
    assert inflation_cells(0.05, 0.1) == 1
    # 非法半径回退到默认 0.15 m -> ceil(0.15/0.1) = 2 格
    assert inflation_cells(0.0, 0.1) == 2


@test
def test_deadline_guard():
    import time

    guard = DeadlineGuard(max_seconds=0.01).start()
    assert not guard.expired()
    time.sleep(0.05)
    assert guard.expired()


def main() -> int:
    passed = 0
    failed = []
    for fn in TESTS:
        try:
            fn()
            passed += 1
            print(f"  PASS  {fn.__name__}")
        except Exception as exc:  # noqa: BLE001
            failed.append((fn.__name__, exc))
            print(f"  FAIL  {fn.__name__}: {exc}")
    print()
    print(f"共 {len(TESTS)} 项，通过 {passed}，失败 {len(failed)}")
    if failed:
        print("\n失败详情：")
        for name, exc in failed:
            print(f"  - {name}: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
