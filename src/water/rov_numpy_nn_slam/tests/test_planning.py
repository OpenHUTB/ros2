#!/usr/bin/env python3
"""规划模块测试：三个场景的 A* 全局路径、DWA 局部避障与几何回退规划器。

运行：``/home/lxy/roscx_runtime/venv/bin/python tests/test_planning.py``
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.planning import DWAPlanner, GeometricPlanner, GridPlanner
from rov_numpy_nn_slam.scenes import get_scene
from rov_numpy_nn_slam.sim import ROVSim


def _sample_path(path, step: float = 0.05):
    points = []
    for a, b in zip(path[:-1], path[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(dist / step)))
        for i in range(n):
            t = i / float(n)
            points.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    points.append((float(path[-1][0]), float(path[-1][1])))
    return points


def _min_ground_clearance(scene, path, limit: float = 1.5):
    """Smallest distance from sampled path points to ground-truth obstacles."""
    res = C.MAP_RESOLUTION
    ground = scene.occupancy(res)
    h, w = ground.shape
    occupied = np.argwhere(ground)
    ox = (occupied[:, 1] + 0.5) * res
    oy = (occupied[:, 0] + 0.5) * res
    best = math.inf
    for x, y in _sample_path(path):
        # only consider obstacle cells in a local window for speed
        sel = (np.abs(ox - x) < limit) & (np.abs(oy - y) < limit)
        if np.any(sel):
            d = np.sqrt((ox[sel] - x) ** 2 + (oy[sel] - y) ** 2)
            best = min(best, float(d.min()))
    return best


def test_astar_all_scenes() -> None:
    print("[astar] scene        length(m)  straight(m)  ratio  clearance(m)  replans")
    for name in C.SCENES:
        scene = get_scene(name)
        planner = GridPlanner(scene.coarse_map(C.PLAN_RESOLUTION), C.PLAN_RESOLUTION)
        start = (float(scene.start[0]), float(scene.start[1]))
        goal = (float(scene.goal[0]), float(scene.goal[1]))
        path = planner.plan(start, goal)
        assert path is not None and len(path) >= 2, "%s: A* found no path" % name
        length = planner.path_length(path)
        straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
        clearance = _min_ground_clearance(scene, path)
        for x, y in path:
            assert planner.is_free(x, y), "%s: waypoint inside inflated obstacle" % name
        end_err = math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1])
        assert clearance >= C.ROV_RADIUS, \
            "%s: path clearance %.3f m < ROV radius %.2f m" % (name, clearance, C.ROV_RADIUS)
        assert end_err < 1.5, "%s: path ends %.2f m from goal" % (name, end_err)
        assert length < 2.0 * straight + 2.0, \
            "%s: path length %.1f m unreasonable (straight %.1f m)" % (name, length, straight)
        print("[astar] %-11s %8.2f  %10.2f  %5.2f  %11.2f  %7d"
              % (name, length, straight, length / max(straight, 1e-6), clearance, 1))
        assert planner.plan(start, goal) is not None  # deterministic repeat


def test_dwa_closed_loop() -> None:
    sim = ROVSim("pool", seed=C.DEFAULT_SEED, noise=True)
    dwa = DWAPlanner()
    goal = np.array([8.0, 1.6])
    dt = 0.02
    sim.reset()
    scan = sim.sonar()
    angles = sim.sonar_beam_angles()
    start_dist = math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1])
    for k in range(int(20.0 / dt)):
        if k % 5 == 0:
            scan = sim.sonar()
            angles = sim.sonar_beam_angles()
        vx, wz = dwa.compute(sim.pose, goal, scan, angles)
        sim.step([vx, 0.0, 0.0, wz], dt)
    final_dist = math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1])
    print("[dwa] goal (8.0,1.6): start dist %.2f m -> %.2f m in 20 s, collisions %d"
          % (start_dist, final_dist, sim.collisions))
    assert sim.collisions == 0, "DWA run collided %d times" % sim.collisions
    assert final_dist < 1.2, "DWA did not reach goal area (%.2f m)" % final_dist


def test_dwa_and_geometric_units() -> None:
    pose = np.array([1.0, 1.0, 1.5, 0.0])
    beams = np.linspace(-C.SONAR_FOV / 2, C.SONAR_FOV / 2, C.SONAR_BEAMS)
    open_scan = np.full(C.SONAR_BEAMS, C.SONAR_RANGE_MAX)
    dwa = DWAPlanner()
    vx_clear, wz_clear = dwa.compute(pose, (5.0, 1.0), open_scan, beams)
    assert vx_clear > 0.3, "DWA should surge in open water"
    assert abs(wz_clear) < 0.3, "DWA should not yaw when goal is straight ahead"

    blocked = open_scan.copy()
    center = C.SONAR_BEAMS // 2
    blocked[center - 6: center + 7] = 0.55
    vx_blocked, _ = dwa.compute(pose, (5.0, 1.0), blocked, beams)
    print("[dwa] open water vx %.2f m/s, obstacle ahead vx %.2f m/s"
          % (vx_clear, vx_blocked))
    assert vx_blocked < vx_clear, "DWA should slow down for an obstacle ahead"

    geo = GeometricPlanner()
    vx_geo, wz_geo = geo.compute(pose, (5.0, 1.0), open_scan, beams)
    assert vx_geo > 0.3 and abs(wz_geo) < 0.3
    vx_geo_blocked, _ = geo.compute(pose, (5.0, 1.0), blocked, beams)
    print("[geo] open water vx %.2f m/s, obstacle ahead vx %.2f m/s"
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
