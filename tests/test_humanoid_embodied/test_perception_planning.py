# -*- coding: utf-8 -*-
"""感知层与规划层的单测（不需要 MuJoCo）。"""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _path  # noqa: E402

from humanoid_embodied_control import perception, planning  # noqa: E402


class TestLidarSectors:
    def test_empty_is_free(self):
        out = perception.lidar_sectors([], 2 * math.pi, 5.0, 8)
        assert out == [1.0] * 8

    def test_all_far_is_free(self):
        out = perception.lidar_sectors([5.0] * 10, 2 * math.pi, 5.0, 4)
        assert all(abs(v - 1.0) < 1e-9 for v in out)

    def test_near_obstacle_gives_low_value(self):
        out = perception.lidar_sectors([0.5] * 10, 2 * math.pi, 5.0, 4)
        assert all(abs(v - 0.1) < 1e-9 for v in out)

    def test_nan_is_treated_as_free(self):
        out = perception.lidar_sectors([float("nan")] * 8, 2 * math.pi, 5.0, 2)
        assert out == [1.0, 1.0]

    def test_takes_min_per_sector(self):
        # 同一扇区里有一个近点 → 整扇取小（不能被远点平均掉）
        out = perception.lidar_sectors([5.0, 5.0, 0.4, 5.0], 2 * math.pi, 5.0, 2)
        assert out[1] < 0.2, "扇区内未取最小值，远处点把障碍稀释了"

    def test_n_sectors_must_be_positive(self):
        with pytest.raises(ValueError):
            perception.lidar_sectors([1.0], 2 * math.pi, 5.0, 0)

    def test_span_and_range_must_be_positive(self):
        with pytest.raises(ValueError):
            perception.lidar_sectors([1.0], 0.0, 5.0, 4)
        with pytest.raises(ValueError):
            perception.lidar_sectors([1.0], 2 * math.pi, 0.0, 4)


class TestDetectObstacles:
    def test_no_obstacle(self):
        assert perception.detect_obstacles([1.0] * 8, 2 * math.pi, 5.0) == []

    def test_single_block_becomes_one_obstacle(self):
        sectors = [1.0, 1.0, 0.2, 0.2, 1.0, 1.0, 1.0, 1.0]
        out = perception.detect_obstacles(sectors, 2 * math.pi, 5.0)
        assert len(out) == 1, "相邻阻塞扇区未合并，会反复绕同一个点"

    def test_sorted_by_distance(self):
        # 两个都低于阈值 0.55 的扇区 → 两个障碍段，按距离升序
        sectors = [0.3, 1.0, 1.0, 1.0, 0.5, 1.0, 1.0, 1.0]
        out = perception.detect_obstacles(sectors, math.pi, 5.0)
        assert len(out) == 2
        assert out[0]["distance"] <= out[1]["distance"]

    def test_one_below_one_above_threshold(self):
        # 0.6 > 阈值 0.55 → 不构成障碍；只有 0.3 那一个
        sectors = [0.3, 1.0, 1.0, 1.0, 0.6, 1.0, 1.0, 1.0]
        out = perception.detect_obstacles(sectors, math.pi, 5.0)
        assert len(out) == 1
        assert abs(out[0]["distance"] - 1.5) < 1e-9

    def test_empty_sectors(self):
        assert perception.detect_obstacles([], 1.0, 5.0) == []

    def test_obstacle_fields(self):
        sectors = [1.0, 0.4, 0.4, 1.0]
        out = perception.detect_obstacles(sectors, math.pi, 5.0)
        assert len(out) == 1
        assert out[0]["width"] == 2
        assert abs(out[0]["distance"] - 2.0) < 1e-9   # min(0.4)*5 = 2.0


class TestFreeDirection:
    def test_picks_most_open(self):
        # 8 扇、scan 范围 2π：最空旷扇 (idx=2, 中心角 3π/8≈67.5°)
        sectors = [0.2, 0.3, 0.9, 0.8, 0.2, 0.3, 0.2, 0.3]
        a = perception.free_direction(sectors, 2 * math.pi)
        assert abs(a - (2 + 0.5) * 2 * math.pi / 8) < 1e-9, a

    def test_tie_breaks_to_front(self):
        # 对称的两个最空旷扇区（1 与 7，中心角 ±157.5°）……它们离正前方一样远，
        # 这里要验证的是"平局时不会随机抖动"：同一输入必须给出同一结果。
        sectors = [0.0, 1.0, 0.2, 0.2, 0.2, 0.2, 0.2, 1.0]
        a = perception.free_direction(sectors, 2 * math.pi)
        b = perception.free_direction(sectors, 2 * math.pi)
        assert a == b, "平局时结果不稳定"
        # 选中的必须是可通行性最大（1.0）的扇区之一
        idx = int(round((a / (2 * math.pi)) * len(sectors) - 0.5))
        assert sectors[idx % len(sectors)] == 1.0

    def test_empty(self):
        assert perception.free_direction([], 2 * math.pi) == 0.0


class TestDepthFree:
    def test_empty_is_free(self):
        assert perception.depth_free_ratio([], 3.0) == 1.0

    def test_near_object(self):
        assert perception.depth_free_ratio([[0.1, 0.2]], 2.0) < 0.2

    def test_invalid_pixels_ignored(self):
        assert perception.depth_free_ratio([[0.0, -1.0, float("nan")]], 3.0) == 1.0

    def test_valid_max_positive(self):
        with pytest.raises(ValueError):
            perception.depth_free_ratio([[1.0]], 0.0)


class TestSensorFusion:
    def test_weighted(self):
        assert abs(perception.sensor_fusion(1.0, 0.0, 0.5) - 0.5) < 1e-9
        assert abs(perception.sensor_fusion(1.0, 0.0, 0.0) - 0.0) < 1e-9

    def test_clamps_weight(self):
        assert perception.sensor_fusion(1.0, 1.0, 5.0) == 1.0

    def test_sectors_mean(self):
        assert perception.sectors_mean([]) == 1.0
        assert abs(perception.sectors_mean([0.2, 0.4]) - 0.3) < 1e-9


class TestFrontClearance:
    def test_middle_index(self):
        sectors = [1.0, 1.0, 0.4, 1.0, 1.0]
        v = perception.front_clearance(sectors, 2 * math.pi, 3.0, 1)
        assert abs(v - 1.2) < 1e-9

    def test_empty(self):
        assert perception.front_clearance([], 1.0, 3.0) == 3.0


# --- planning -------------------------------------------------------
class TestCostMap:
    def _cm(self):
        return planning.CostMap(20, 20, 0.5)

    def test_coord_roundtrip(self):
        cm = self._cm()
        x, y = cm.cell_to_world(4, 6)
        r, c = cm.world_to_cell(x, y)
        assert (r, c) == (4, 6)

    def test_bounds(self):
        cm = self._cm()
        assert cm.is_free(0, 0)
        assert not cm.is_free(-1, 0)
        assert not cm.is_free(0, 99)

    def test_add_circle(self):
        cm = self._cm()
        cm.add_circle(5.0, 5.0, 0.8)
        assert not cm.is_free(10, 10)
        assert cm.is_free(0, 0)

    def test_add_circle_zero_radius_noop(self):
        cm = self._cm()
        cm.add_circle(5.0, 5.0, 0.0)
        assert cm.is_free(10, 10)

    def test_clearance(self):
        cm = self._cm()
        assert cm.clearance(2.0, 2.0, 0.5) == 1.0
        cm.add_circle(2.2, 2.2, 0.3)
        assert cm.clearance(2.0, 2.0, 0.8) == 0.0

    def test_invalid_ctor(self):
        for kwargs in (dict(width=0, height=10, res=0.5),
                       dict(width=10, height=0, res=0.5),
                       dict(width=10, height=10, res=0.0)):
            with pytest.raises(ValueError):
                planning.CostMap(**kwargs)

    def test_occupied_ratio(self):
        cm = self._cm()
        cm.set_occupied(0, 0)
        assert cm.occupied_ratio() > 0.0


class TestAStar:
    def test_straight_line(self):
        cm = planning.CostMap(20, 20, 0.5)
        p = planning.astar(cm, (10, 2), (10, 17))
        assert p and p[0] == (10, 2) and p[-1] == (10, 17)

    def test_blocked_returns_empty(self):
        cm = planning.CostMap(20, 20, 0.5)
        for col in range(0, 20):
            cm.set_occupied(10, col)
        assert planning.astar(cm, (5, 5), (15, 5)) == []

    def test_no_corner_cutting(self):
        cm = planning.CostMap(20, 20, 0.5)
        cm.set_occupied(9, 9)
        cm.set_occupied(10, 10)
        p = planning.astar(cm, (10, 9), (9, 10))
        assert p, "切角被禁后仍应有绕行路径"
        for (r1, c1), (r2, c2) in zip(p, p[1:]):
            assert not (abs(r1 - r2) + abs(c1 - c2) == 2 and
                        (not cm.is_free(r1, c2) or not cm.is_free(r2, c1)))

    def test_goal_on_obstacle_fails(self):
        cm = planning.CostMap(20, 20, 0.5)
        cm.set_occupied(10, 10)
        assert planning.astar(cm, (10, 11), (10, 10)) == []

    def test_same_start_goal(self):
        cm = planning.CostMap(20, 20, 0.5)
        assert planning.astar(cm, (5, 5), (5, 5)) == [(5, 5)]


class TestPathHelpers:
    def test_path_length(self):
        assert planning.path_length([(0, 0), (3, 0), (3, 4)]) == 7.0
        assert planning.path_length([]) == 0.0
        assert planning.path_length([(1, 1)]) == 0.0

    def test_path_to_world(self):
        cm = planning.CostMap(10, 10, 0.5)
        pts = planning.path_to_world(cm, [(1, 1)])
        assert len(pts) == 1 and len(pts[0]) == 2

    def test_lookahead_forward(self):
        p = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        got = planning.lookahead_point(p, 0.0, 0.0, 1.5)
        assert abs(got[0] - 2.0) < 1e-9

    def test_lookahead_beyond_returns_last(self):
        p = [(0.0, 0.0), (1.0, 0.0)]
        assert planning.lookahead_point(p, 0.0, 0.0, 5.0) == p[-1]

    def test_lookahead_empty(self):
        assert planning.lookahead_point([], 0.0, 0.0, 1.0) is None

    def test_heading_error_wrapped(self):
        e = planning.heading_error(0.0, 0.0, math.pi - 0.01, (1.0, 0.0))
        assert -math.pi <= e <= math.pi

    def test_replan_needed(self):
        # 阈值：|heading|>0.6 或 front < obstacle_dist*0.45
        assert planning.replan_needed(0.1, 0.8, 1.0) is False
        assert planning.replan_needed(0.9, 0.8, 1.0) is True
        assert planning.replan_needed(0.1, 0.8, 0.3) is True
