#!/usr/bin/env python3
"""Planning tests: workspace A* paths for all MSK-Arm scenes.

Run: ``/home/lxy/roscx_runtime/venv/bin/python tests/test_planning.py``

The end-effector paths are validated against the exact workspace occupancy grid
(at ``config.PLAN_RESOLUTION``) and against the raw scene geometry (end-effector
radius); the ``draw`` scene gets one A* segment per target.  A target that sits
closer than the inflation radius to an obstacle is planned to the nearest free
end-effector point instead - the muscle controller then satisfies it within
``config.TARGET_TOLERANCE``.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from msk_arm_nn import config as C
from msk_arm_nn.control import forward_kinematics
from msk_arm_nn.planning import (GridPlanner, densify_path, plan_sequence,
                                 polyline_length, scene_planning_grid)
from msk_arm_nn.scenes import get_scene

_INFLATE_M = C.EE_RADIUS + 0.005     # >= EE_RADIUS as required


def _start_ee(scene) -> tuple:
    ee = forward_kinematics(scene.start_q)
    return float(ee[0]), float(ee[1])


def _obstacle_clearance(scene, x: float, y: float) -> float:
    best = math.inf
    for cx, cy, hw, hh in scene.obstacles:
        dx = max(abs(x - cx) - hw, 0.0)
        dy = max(abs(y - cy) - hh, 0.0)
        best = min(best, math.hypot(dx, dy))
    if not math.isfinite(best):
        return float(C.WORK_X[1] - C.WORK_X[0])
    return best


def test_astar_all_scenes() -> None:
    print("[astar] scene     target #  length(m)  straight(m)  ratio  "
          "clearance(m)  waypoints")
    res = C.PLAN_RESOLUTION
    expected = (int(round((C.WORK_Y[1] - C.WORK_Y[0]) / res)),
                int(round((C.WORK_X[1] - C.WORK_X[0]) / res)))
    for name in C.SCENES:
        scene = get_scene(name)
        grid = scene_planning_grid(scene, res)
        assert grid.shape == expected, \
            "%s: planning grid shape %s != %s" % (name, grid.shape, expected)
        planner = GridPlanner(grid, res,
                              inflate_cells=_INFLATE_M / res)
        assert planner.inflate_radius_m >= C.EE_RADIUS
        start = _start_ee(scene)
        raw_truth = np.asarray(scene.occupancy(res), dtype=bool)
        for i, target in enumerate(scene.targets):
            goal = planner.nearest_free(float(target[0]), float(target[1]))
            segment = planner.plan(start, goal)
            assert segment is not None and len(segment) >= 2, \
                "%s: A* found no path to target %d" % (name, i)
            length = polyline_length(segment)
            straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
            for x, y in densify_path(segment, res * 0.5):
                c = int(math.floor((x - C.WORK_X[0]) / res))
                r = int(math.floor((y - C.WORK_Y[0]) / res))
                c = min(max(c, 0), grid.shape[1] - 1)
                r = min(max(r, 0), grid.shape[0] - 1)
                assert not raw_truth[r, c], \
                    "%s: path enters a blocked workspace cell (%.3f, %.3f)" % (
                        name, x, y)
            clearance = math.inf
            for a, b in zip(segment[:-1], segment[1:]):
                for x, y in densify_path([a, b], 0.004):
                    assert _obstacle_clearance(scene, x, y) >= 0.5 * C.EE_RADIUS, \
                        "%s: path passes through an obstacle" % name
                    clearance = min(clearance, _obstacle_clearance(scene, x, y))
            assert clearance >= 0.8 * C.EE_RADIUS, \
                "%s: path clearance %.3f m < EE radius %.3f m" % (
                    name, clearance, C.EE_RADIUS)
            end_err = math.hypot(segment[-1][0] - goal[0],
                                 segment[-1][1] - goal[1])
            assert end_err < res * 1.5, \
                "%s: path ends %.3f m from the planned goal" % (name, end_err)
            snap = math.hypot(goal[0] - target[0], goal[1] - target[1])
            assert snap <= C.TARGET_TOLERANCE + res, \
                "%s: target %d is %.3f m inside the inflated workspace" % (
                    name, i, snap)
            assert length < 3.0 * straight + 0.30, \
                "%s: path length %.3f m unreasonable (straight %.3f m)" % (
                    name, length, straight)
            print("[astar] %-8s %6d  %8.3f  %10.3f  %5.2f  %11.3f  %9d"
                  "  target_snap=%.3f"
                  % (name, i, length, straight,
                     length / max(straight, 1e-6), clearance, len(segment),
                     snap))
            start = (float(target[0]), float(target[1]))


def test_planner_units() -> None:
    scene = get_scene("shelf")
    res = C.PLAN_RESOLUTION
    planner = GridPlanner(scene_planning_grid(scene, res), res)
    # the shelf obstacle must be blocked, free workspace must be free
    cx, cy, hw, hh = scene.obstacles[0]
    assert not planner.is_free(cx, cy), "shelf obstacle should be blocked"
    assert planner.is_free(0.20, 0.10), "free workspace should be free"
    assert planner.inflate_radius_m >= C.EE_RADIUS
    path = planner.plan((0.45, 0.10), (0.45, 0.35))
    assert path is not None
    assert planner.path_length(path) >= 0.24
    assert len(path) <= 8, "LOS smoothing should collapse the grid path"
    print("[planner] obstacle blocked / inflated %.3f m / LOS waypoints %d OK"
          % (planner.inflate_radius_m, len(path)))


def main() -> int:
    print("== test_planning ==")
    test_astar_all_scenes()
    test_planner_units()
    print("[planning] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
