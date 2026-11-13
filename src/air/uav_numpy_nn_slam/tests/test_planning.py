#!/usr/bin/env python3
"""Planning tests: A* in all three aerial scenes, DWA closed loop and units.

Run: ``python3 tests/test_planning.py``

The A* paths are validated against ``scene.planning_map()`` truth (buildings and
no-fly zones are both blocked) and against the raw building occupancy for
clearance; the DWA run flies a real ``UAVSim`` in the city and must not collide.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from uav_numpy_nn_slam import config as C
from uav_numpy_nn_slam.planning import DWAPlanner, GeometricPlanner, GridPlanner
from uav_numpy_nn_slam.scenes import get_scene
from uav_numpy_nn_slam.sim import UAVSim


def _beam_angles() -> np.ndarray:
    """Full-circle lidar beam angles, matching ``UAVSim.lidar_beam_angles``."""
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


def _min_ground_clearance(scene, path, limit: float = 3.0) -> float:
    """Smallest distance from sampled path points to ground-truth occupied cells."""
    res = C.MAP_RESOLUTION
    ground = scene.occupancy(res)
    h, w = ground.shape
    k = int(math.ceil(limit / res))
    best = math.inf
    for x, y in _sample_path(path, 0.40):
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


def _endpoints(planner, scene, notes):
    """Start/goal in planning-grid terms; snap out of blocked cells if needed."""
    sx, sy = float(scene.start[0]), float(scene.start[1])
    gx, gy = float(scene.goal[0]), float(scene.goal[1])
    sc = planner.world_to_cell(sx, sy)
    gc = planner.world_to_cell(gx, gy)
    if planner.grid[sc[1], sc[0]]:
        sx, sy = planner.nearest_free(sx, sy)
        notes.append("%s scene.start is blocked -> snapped %.1f,%.1f"
                     % (scene.name, sx, sy))
    if planner.grid[gc[1], gc[0]]:
        gx, gy = planner.nearest_free(gx, gy)
        notes.append("%s scene.goal is blocked -> snapped %.1f,%.1f"
                     % (scene.name, gx, gy))
    return (sx, sy), (gx, gy)


def test_astar_all_scenes() -> None:
    print("[astar] scene      length(m)  straight(m)  ratio  clearance(m)  "
          "nfz_clear(m)  waypoints")
    notes = []
    for name in C.SCENES:
        scene = get_scene(name)
        grid = np.asarray(scene.planning_map(C.PLAN_RESOLUTION), dtype=bool)
        planner = GridPlanner(grid, C.PLAN_RESOLUTION)
        start, goal = _endpoints(planner, scene, notes)
        path = planner.plan(start, goal)
        assert path is not None and len(path) >= 2, "%s: A* found no path" % name
        length = planner.path_length(path)
        straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
        clearance = _min_ground_clearance(scene, path)
        nfz_clear = min(scene.nfz_clearance(x, y)
                        for x, y in _sample_path(path, 0.40))
        for x, y in path:
            assert planner.is_free(x, y), \
                "%s: waypoint (%.1f, %.1f) inside inflated obstacle" % (name, x, y)
        # ground truth: no sampled point may sit in a planning_map blocked cell
        # (buildings + no-fly zones) nor inside a no-fly zone
        h, w = grid.shape
        for x, y in _sample_path(path, 0.40):
            c = min(max(int(x / C.PLAN_RESOLUTION), 0), w - 1)
            r = min(max(int(y / C.PLAN_RESOLUTION), 0), h - 1)
            assert not grid[r, c], \
                "%s: path enters planning-map blocked cell (%.1f, %.1f)" % (
                    name, x, y)
            assert not scene.inside_nfz(x, y), \
                "%s: path enters a no-fly zone (%.1f, %.1f)" % (name, x, y)
        end_err = math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1])
        assert clearance >= C.UAV_RADIUS, \
            "%s: path clearance %.3f m < UAV radius %.2f m" % (
                name, clearance, C.UAV_RADIUS)
        assert nfz_clear >= 0.5, \
            "%s: path comes %.2f m from a no-fly zone" % (name, nfz_clear)
        assert end_err < 1.5, "%s: path ends %.2f m from goal" % (name, end_err)
        assert length < 2.5 * straight + 2.0, \
            "%s: path length %.1f m unreasonable (straight %.1f m)" % (
                name, length, straight)
        print("[astar] %-8s %8.2f  %10.2f  %5.2f  %11.2f  %11.2f  %9d"
              % (name, length, straight, length / max(straight, 1e-6),
                 clearance, nfz_clear, len(path)))
    for note in notes:
        print("[astar] WARNING: %s (scene geometry bug in scenes.py)" % note)


def test_dwa_closed_loop() -> None:
    sim = UAVSim("city", seed=C.DEFAULT_SEED, noise=True)
    dwa = DWAPlanner(max_surge=3.0)
    goal = np.array([43.0, 30.0])
    dt = 0.05
    sim.reset()
    scan = sim.lidar()
    angles = sim.lidar_beam_angles()
    start_dist = math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1])
    t = 0.0
    for k in range(int(25.0 / dt)):
        if k % 4 == 0:
            scan = sim.lidar()
            angles = sim.lidar_beam_angles()
        vx, wz = dwa.compute(sim.pose, goal, scan, angles)
        yaw = float(sim.pose[3])
        # DWA returns a body-frame forward speed; the sim tracks world velocities
        sim.step([vx * math.cos(yaw), vx * math.sin(yaw), 0.0, wz], dt)
        t += dt
        if math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1]) < 3.0:
            break
    final_dist = math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1])
    print("[dwa] goal (43.0, 30.0): %.2f m -> %.2f m in %.1f s, collisions %d"
          % (start_dist, final_dist, t, sim.collisions))
    assert sim.collisions == 0, "DWA run collided %d times" % sim.collisions
    assert final_dist < start_dist - 8.0, \
        "DWA made too little progress (%.2f m)" % final_dist


def test_dwa_and_geometric_units() -> None:
    pose = np.array([1.0, 1.0, 20.0, 0.0])
    beams = _beam_angles()
    open_scan = np.full(C.LIDAR_BEAMS, C.LIDAR_RANGE_MAX)
    dwa = DWAPlanner()
    vx_clear, wz_clear = dwa.compute(pose, (50.0, 1.0), open_scan, beams)
    assert vx_clear > 0.3, "DWA should surge in open air"
    assert abs(wz_clear) < 0.3, "DWA should not yaw when the goal is dead ahead"

    blocked = open_scan.copy()
    center = C.LIDAR_BEAMS // 2
    blocked[center - 6: center + 7] = 2.0
    vx_blocked, _ = dwa.compute(pose, (50.0, 1.0), blocked, beams)
    print("[dwa] open air vx %.2f m/s, obstacle ahead vx %.2f m/s"
          % (vx_clear, vx_blocked))
    assert vx_blocked < vx_clear, "DWA should slow down for an obstacle ahead"

    geo = GeometricPlanner()
    vx_geo, wz_geo = geo.compute(pose, (50.0, 1.0), open_scan, beams)
    assert vx_geo > 0.3 and abs(wz_geo) < 0.3
    vx_geo_blocked, _ = geo.compute(pose, (50.0, 1.0), blocked, beams)
    print("[geo] open air vx %.2f m/s, obstacle ahead vx %.2f m/s"
          % (vx_geo, vx_geo_blocked))
    assert vx_geo_blocked < vx_geo


def main() -> int:
    print("== test_planning ==")
    test_astar_all_scenes()
    test_dwa_closed_loop()
    test_dwa_and_geometric_units()
    print("[planning] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
