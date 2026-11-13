#!/usr/bin/env python3
"""Planning tests: A* on the three indoor scenes, DWA closed loop and units.

Run: ``/home/lxy/roscx_runtime/venv/bin/python tests/test_planning.py``

The A* paths are validated against the exact floor-plan occupancy grid at
``config.PLAN_RESOLUTION`` (walls, crates and pillars blocked) and their
clearance against the raw scene occupancy; the DWA run walks a real
``HumanoidSim`` and must not collide or fall.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from humanoid_walk_nn import config as C
from humanoid_walk_nn.planning import (DWAPlanner, GeometricPlanner,
                                       GridPlanner, scene_planning_grid)
from humanoid_walk_nn.scenes import get_scene
from humanoid_walk_nn.sim import HumanoidSim


def _beam_angles() -> np.ndarray:
    """Full-circle lidar beam angles, matching ``HumanoidSim.lidar_beam_angles``."""
    return np.linspace(-math.pi, math.pi, C.LIDAR_BEAMS, endpoint=False)


def _sample_path(path, step: float = 0.20):
    points = []
    for a, b in zip(path[:-1], path[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(dist / step)))
        for i in range(n):
            t = i / float(n)
            points.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    points.append((float(path[-1][0]), float(path[-1][1])))
    return points


def _min_ground_clearance(scene, path, limit: float = 2.0) -> float:
    """Smallest distance from sampled path points to ground-truth blocked cells."""
    res = C.MAP_RESOLUTION
    ground = scene.occupancy(res)
    h, w = ground.shape
    k = int(math.ceil(limit / res))
    best = math.inf
    for x, y in _sample_path(path, 0.20):
        c = int(x / res)
        r = int(y / res)
        c0, c1 = max(0, c - k), min(w, c + k + 1)
        r0, r1 = max(0, r - k), min(h, r + k + 1)
        window = ground[r0:r1, c0:c1]
        if not window.any():
            continue
        rr, cc = np.nonzero(window)
        d = np.hypot((cc + c0 - c) * res, (rr + r0 - r) * res)
        best = min(best, float(d.min()))
    return best


def test_astar_all_scenes() -> None:
    print("[astar] scene        length(m)  straight(m)  ratio  clearance(m)  "
          "waypoints")
    res = C.PLAN_RESOLUTION
    for name in C.SCENES:
        scene = get_scene(name)
        grid = scene_planning_grid(scene, res)
        expected = (int(round(scene.height / res)),
                    int(round(scene.width / res)))
        assert grid.shape == expected, \
            "%s: planning grid shape %s != %s" % (name, grid.shape, expected)
        planner = GridPlanner(grid, res,
                              inflate_cells=C.INFLATION_RADIUS / res)
        start = (float(scene.start[0]), float(scene.start[1]))
        goal = (float(scene.goal[0]), float(scene.goal[1]))
        path = planner.plan(start, goal)
        assert path is not None and len(path) >= 2, \
            "%s: A* found no path (scene geometry impassable?)" % name
        length = planner.path_length(path)
        straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
        clearance = _min_ground_clearance(scene, path)
        h, w = grid.shape
        for x, y in _sample_path(path, 0.20):
            c = min(max(int(x / res), 0), w - 1)
            r = min(max(int(y / res), 0), h - 1)
            assert not grid[r, c], \
                "%s: path enters a blocked floor-plan cell (%.1f, %.1f)" % (
                    name, x, y)
        for x, y in path:
            assert planner.is_free(x, y), \
                "%s: waypoint (%.1f, %.1f) inside inflated obstacle" % (
                    name, x, y)
        end_err = math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1])
        assert clearance >= C.ROBOT_RADIUS, \
            "%s: path clearance %.2f m < robot radius %.2f m" % (
                name, clearance, C.ROBOT_RADIUS)
        assert end_err < 0.5, "%s: path ends %.2f m from goal" % (name, end_err)
        assert length < 2.5 * straight + 1.0, \
            "%s: path length %.1f m unreasonable (straight %.1f m)" % (
                name, length, straight)
        print("[astar] %-10s %8.2f  %10.2f  %5.2f  %11.2f  %9d"
              % (name, length, straight, length / max(straight, 1e-6),
                 clearance, len(path)))


def test_dwa_closed_loop() -> None:
    sim = HumanoidSim("hall", seed=C.DEFAULT_SEED, noise=True)
    dwa = DWAPlanner(scene=sim.scene)
    goal = np.array([28.0, 6.0])
    dt = 0.05
    scan = sim.lidar()
    angles = sim.lidar_beam_angles()
    start_dist = math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1])
    t = 0.0
    final_dist = start_dist
    for k in range(int(25.0 / dt)):
        if k % 4 == 0:
            scan = sim.lidar()
            angles = sim.lidar_beam_angles()
        v, omega = dwa.compute(sim.pose, goal, scan, angles,
                               speed=sim.speed, dt=dt)
        sim.step([v, omega], dt)
        t += dt
        final_dist = math.hypot(sim.pose[0] - goal[0],
                                sim.pose[1] - goal[1])
        if final_dist < 1.0:
            break
    print("[dwa] goal (28.0, 6.0): %.2f m -> %.2f m in %.1f s, "
          "collisions %d, falls %d"
          % (start_dist, final_dist, t, sim.collisions, sim.falls))
    assert sim.collisions == 0, "DWA run collided %d times" % sim.collisions
    assert sim.falls == 0, "DWA run fell %d times" % sim.falls
    assert final_dist < start_dist - 6.0, \
        "DWA made too little progress (%.2f m)" % final_dist


def test_dwa_and_geometric_units() -> None:
    pose = np.array([2.0, 6.0, 0.0, 0.0])
    beams = _beam_angles()
    open_scan = np.full(C.LIDAR_BEAMS, C.LIDAR_RANGE_MAX)
    dwa = DWAPlanner()
    v_clear, w_clear = dwa.compute(pose, (10.0, 6.0), open_scan, beams,
                                   speed=0.0)
    assert v_clear > 0.2, "DWA should walk forward on an open corridor"
    assert abs(w_clear) < 0.3, "DWA should not turn for a straight goal"

    blocked = open_scan.copy()
    center = C.LIDAR_BEAMS // 2
    blocked[center - 4: center + 5] = 0.30
    v_blocked, _ = dwa.compute(pose, (10.0, 6.0), blocked, beams, speed=0.0)
    print("[dwa] open corridor v %.2f m/s, obstacle ahead v %.2f m/s"
          % (v_clear, v_blocked))
    assert v_blocked <= 0.05, "DWA must not walk into an obstacle ahead"

    geo = GeometricPlanner()
    v_geo, w_geo = geo.compute(pose, (10.0, 6.0), open_scan, beams)
    assert v_geo > 0.2 and abs(w_geo) < 0.3
    v_geo_blocked, _ = geo.compute(pose, (10.0, 6.0), blocked, beams)
    print("[geo] open corridor v %.2f m/s, obstacle ahead v %.2f m/s"
          % (v_geo, v_geo_blocked))
    assert v_geo_blocked < v_geo


def main() -> int:
    print("== test_planning ==")
    test_astar_all_scenes()
    test_dwa_closed_loop()
    test_dwa_and_geometric_units()
    print("[planning] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
