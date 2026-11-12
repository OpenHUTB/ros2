#!/usr/bin/env python3
"""Planning tests: A* on the three road scenes, DWA closed loop and units.

Run: ``/home/lxy/roscx_runtime/venv/bin/python tests/test_planning.py``

The A* paths are validated against the exact road occupancy grid at
``config.PLAN_RESOLUTION`` (off-road, parked cars, cones and barriers are all
blocked) and their clearance against the raw scene occupancy; the DWA run
drives a real ``RoadSim`` and must not collide.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from road_numpy_nn_slam import config as C
from road_numpy_nn_slam.planning import (DWAPlanner, GeometricPlanner, GridPlanner,
                                  scene_planning_grid)
from road_numpy_nn_slam.scenes import get_scene
from road_numpy_nn_slam.sim import RoadSim


def _beam_angles() -> np.ndarray:
    """Full-circle lidar beam angles, matching ``RoadSim.lidar_beam_angles``."""
    return np.linspace(-math.pi, math.pi, C.LIDAR_BEAMS, endpoint=False)


def _sample_path(path, step: float = 0.25):
    points = []
    for a, b in zip(path[:-1], path[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(dist / step)))
        for i in range(n):
            t = i / float(n)
            points.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    points.append((float(path[-1][0]), float(path[-1][1])))
    return points


def _min_ground_clearance(scene, path, limit: float = 3.0) -> float:
    """Smallest distance from sampled path points to ground-truth blocked cells."""
    res = C.MAP_RESOLUTION
    ground = scene.occupancy(res)
    h, w = ground.shape
    k = int(math.ceil(limit / res))
    best = math.inf
    for x, y in _sample_path(path, 0.5):
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
        for x, y in _sample_path(path, 0.4):
            c = min(max(int(x / res), 0), w - 1)
            r = min(max(int(y / res), 0), h - 1)
            assert not grid[r, c], \
                "%s: path enters a blocked planning cell (%.1f, %.1f)" % (
                    name, x, y)
        for x, y in path:
            assert planner.is_free(x, y), \
                "%s: waypoint (%.1f, %.1f) inside inflated obstacle" % (
                    name, x, y)
        end_err = math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1])
        assert clearance >= C.CAR_RADIUS, \
            "%s: path clearance %.2f m < car radius %.2f m" % (
                name, clearance, C.CAR_RADIUS)
        assert end_err < 1.5, "%s: path ends %.2f m from goal" % (name, end_err)
        assert length < 2.5 * straight + 2.0, \
            "%s: path length %.1f m unreasonable (straight %.1f m)" % (
                name, length, straight)
        print("[astar] %-10s %8.2f  %10.2f  %5.2f  %11.2f  %9d"
              % (name, length, straight, length / max(straight, 1e-6),
                 clearance, len(path)))


def test_dwa_closed_loop() -> None:
    sim = RoadSim("downtown", seed=C.DEFAULT_SEED, noise=True)
    dwa = DWAPlanner(scene=sim.scene)
    goal = np.array([28.0, 70.0])
    dt = 0.05
    scan = sim.lidar()
    angles = sim.lidar_beam_angles()
    start_dist = math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1])
    t = 0.0
    final_dist = start_dist
    for k in range(int(15.0 / dt)):
        if k % 4 == 0:
            scan = sim.lidar()
            angles = sim.lidar_beam_angles()
        throttle, steer = dwa.compute(sim.pose, goal, scan, angles,
                                      speed=sim.speed, dt=dt)
        sim.step([throttle, steer], dt)
        t += dt
        final_dist = math.hypot(sim.pose[0] - goal[0],
                                sim.pose[1] - goal[1])
        if final_dist < 3.0:
            break
    print("[dwa] goal (28.0, 70.0): %.2f m -> %.2f m in %.1f s, "
          "collisions %d, red-light %d"
          % (start_dist, final_dist, t, sim.collisions,
             sim.red_light_violations))
    assert sim.collisions == 0, "DWA run collided %d times" % sim.collisions
    assert final_dist < start_dist - 8.0, \
        "DWA made too little progress (%.2f m)" % final_dist


def test_dwa_and_geometric_units() -> None:
    pose = np.array([1.0, 1.0, 0.0, 0.0])
    beams = _beam_angles()
    open_scan = np.full(C.LIDAR_BEAMS, C.LIDAR_RANGE_MAX)
    dwa = DWAPlanner()
    throttle_clear, steer_clear = dwa.compute(pose, (30.0, 1.0), open_scan,
                                              beams, speed=0.0)
    assert throttle_clear > 0.2, "DWA should accelerate on an open road"
    assert abs(steer_clear) < 0.3, "DWA should not steer for a straight goal"

    blocked = open_scan.copy()
    center = C.LIDAR_BEAMS // 2
    blocked[center - 6: center + 7] = 0.8
    throttle_blocked, _ = dwa.compute(pose, (30.0, 1.0), blocked, beams,
                                      speed=0.0)
    print("[dwa] open road throttle %.2f, obstacle ahead throttle %.2f"
          % (throttle_clear, throttle_blocked))
    assert throttle_blocked <= 0.0, "DWA should brake for an obstacle ahead"

    geo = GeometricPlanner()
    throttle_geo, steer_geo = geo.compute(pose, (30.0, 1.0), open_scan, beams,
                                          speed=0.0)
    assert throttle_geo > 0.2 and abs(steer_geo) < 0.3
    throttle_geo_blocked, _ = geo.compute(pose, (30.0, 1.0), blocked, beams,
                                          speed=0.0)
    print("[geo] open road throttle %.2f, obstacle ahead throttle %.2f"
          % (throttle_geo, throttle_geo_blocked))
    assert throttle_geo_blocked < throttle_geo


def main() -> int:
    print("== test_planning ==")
    test_astar_all_scenes()
    test_dwa_closed_loop()
    test_dwa_and_geometric_units()
    print("[planning] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
