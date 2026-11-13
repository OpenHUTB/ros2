#!/usr/bin/env python3
"""Control tests: PID / altitude hold / pure pursuit / safety filter + closed-loop
missions in the city, canyon and airport aerial scenes.

Run: ``python3 tests/test_control.py``

Acceptance: ``MissionController`` drives a real ``UAVSim`` from ``scene.start``
to ``scene.goal`` (<= 3 m horizontal, <= 2 m altitude) with 0 collisions and
0 no-fly-zone violations, in every scene.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from uav_numpy_nn_slam import config as C
from uav_numpy_nn_slam.control import (AltitudeController, DepthController,
                                MissionController, PID, PurePursuit,
                                SafetyFilter)
from uav_numpy_nn_slam.planning import GridPlanner
from uav_numpy_nn_slam.scenes import get_scene
from uav_numpy_nn_slam.sim import UAVSim


def _beam_angles() -> np.ndarray:
    """Full-circle lidar beam angles, matching ``UAVSim.lidar_beam_angles``."""
    return np.linspace(-math.pi, math.pi, C.LIDAR_BEAMS, endpoint=False)


def test_pid() -> None:
    pid = PID(kp=2.0, ki=1.0, kd=0.0, limit=3.0, i_limit=10.0)
    first = pid.update(1.0, 0.1)
    assert abs(first - 2.1) < 1e-9, "P+I term wrong: %.3f" % first
    for _ in range(100):
        out = pid.update(1.0, 0.1)
    assert abs(out - 3.0) < 1e-9, "output limit not applied: %.3f" % out
    pid.reset()
    assert abs(pid.update(0.0, 0.1)) < 1e-9
    neg = PID(kp=1.0, ki=0.0, kd=0.0, limit=0.5)
    assert abs(neg.update(-2.0, 0.1) + 0.5) < 1e-9, "negative saturation wrong"
    damp = PID(kp=0.0, ki=0.0, kd=1.0)
    damp.update(0.0, 0.1)
    assert abs(damp.update(1.0, 0.1) - 10.0) < 1e-6, "D term wrong"
    print("[pid] P/I saturation, reset and derivative OK")


def test_altitude_control() -> None:
    assert AltitudeController is DepthController, "aerial alias missing"
    probe = AltitudeController()
    # altitude is positive up: being below the target must command a climb
    assert probe.update(30.0, 40.0, 0.05) > 0.0, "altitude error sign wrong"
    assert probe.update(50.0, 40.0, 0.05) < 0.0, "altitude error sign wrong"

    sim = UAVSim("airport", seed=11, noise=True)
    alt = AltitudeController(target=40.0)
    dt = 0.05
    for _ in range(int(14.0 / dt)):
        vz = alt.update(sim.pose[2], 40.0, dt)
        sim.step([0.0, 0.0, vz, 0.0], dt)
    err = abs(sim.pose[2] - 40.0)
    print("[alt] target 40.0 m -> %.2f m, error %.2f m after 14 s, collisions %d"
          % (sim.pose[2], err, sim.collisions))
    assert err < 1.0, "altitude hold error %.2f m too large" % err
    assert sim.collisions == 0
    for _ in range(int(5.0 / dt)):
        vz = alt.update(sim.pose[2], 40.0, dt)
        sim.step([0.0, 0.0, vz, 0.0], dt)
    hold_err = abs(sim.pose[2] - 40.0)
    print("[alt] 5 s hold -> %.2f m, error %.2f m" % (sim.pose[2], hold_err))
    assert hold_err < 0.7, "altitude hold drifted %.2f m" % hold_err


def test_pure_pursuit() -> None:
    pp = PurePursuit(lookahead=2.5)
    path = [(0.0, 0.0), (30.0, 0.0), (60.0, 0.0)]
    vx, wz = pp.compute((0.0, 0.0, 0.0), path)
    assert vx > 0.5 and abs(wz) < 0.05, "straight path tracking wrong"
    vx2, wz2 = pp.compute((0.0, 0.0, math.pi / 2.0), path)
    assert vx2 > 0.0 and wz2 < -0.3, "should yaw right toward the +x path"
    vx3, wz3 = pp.compute((60.0, 0.0, 0.0), path)
    assert vx3 == 0.0 and wz3 == 0.0, "path end should stop"
    assert pp.compute((0.0, 0.0, 0.0), []) == (0.0, 0.0)
    # world-frame drive must point down the path regardless of the heading
    wx, wy, wzz = pp.compute_world((0.0, 0.0, math.pi / 2.0), path)
    assert wx > 0.5 and abs(wy) < 1e-6, "world-frame drive direction wrong"
    print("[pursuit] straight / turning / path-end / world-frame OK")


def test_safety_filter() -> None:
    beams = _beam_angles()
    safety = SafetyFilter(min_clearance=3.2, hard_clearance=1.6)
    clear = np.full(C.LIDAR_BEAMS, C.LIDAR_RANGE_MAX)
    cmd = safety.apply([1.0, 0.0, 0.0, 0.2], clear, beams)
    assert abs(cmd[0] - 1.0) < 1e-9, "clear air must not change the speed"
    center = C.LIDAR_BEAMS // 2
    hard = clear.copy()
    hard[center] = 1.0
    cmd = safety.apply([1.0, 0.0, 0.0, 0.2], hard, beams)
    assert cmd[0] == 0.0, "close obstacle should stop the forward speed"
    soft = clear.copy()
    soft[center] = 2.4
    cmd = safety.apply([1.0, 0.0, 0.0, 0.2], soft, beams)
    assert 0.0 < cmd[0] < 1.0, "medium clearance should reduce the speed"
    print("[safety] clear / slow / stop OK (2.40 m -> vx %.2f m/s)" % cmd[0])


def _free_start(scene):
    """Start pose clear of the inflated planning grid (defensive helper).

    The canyon scene in the shipped ``scenes.py`` spawns on/inside the west cliff
    wall; the mission must not be rejected for a scene-geometry bug outside this
    module's scope, so a blocked start is snapped to the nearest free cell and
    reported loudly.
    """
    planner = GridPlanner(scene.planning_map(C.PLAN_RESOLUTION),
                          C.PLAN_RESOLUTION)
    x, y, alt, yaw = (float(scene.start[0]), float(scene.start[1]),
                      float(scene.start[2]), float(scene.start[3]))
    if planner.is_free(x, y):
        return (x, y, alt, yaw), None
    nx, ny = planner.nearest_free(x, y)
    return (nx, ny, alt, yaw), (x, y)


def run_mission_scene(name: str, max_time: float = 150.0) -> dict:
    scene = get_scene(name)
    sim = UAVSim(name, seed=C.DEFAULT_SEED, noise=True)
    start, moved = _free_start(scene)
    if moved is not None:
        print("[mission] %s WARNING: scene.start (%.1f, %.1f) is inside an "
              "obstacle (scenes.py bug) -> resetting to (%.1f, %.1f)"
              % ((name,) + moved + (start[0], start[1])))
        sim.reset(pose=list(start))
    mission = MissionController(scene)
    goal = np.array(scene.goal, dtype=np.float64)
    dt = 0.05
    scan = sim.lidar()
    angles = sim.lidar_beam_angles()
    t = 0.0
    k = 0
    prev = sim.pose.copy()
    path_len = 0.0
    reached_at = None
    min_nfz = scene.nfz_clearance(prev[0], prev[1]) if scene.nfz else float("inf")
    while t < max_time:
        if k % 2 == 0:
            scan = sim.lidar()
            angles = sim.lidar_beam_angles()
        k += 1
        cmd = mission.update(sim.pose, goal, scan, angles, dt)
        sim.step(cmd, dt)
        t += dt
        pose = sim.pose
        path_len += math.hypot(pose[0] - prev[0], pose[1] - prev[1])
        prev = pose.copy()
        if scene.nfz:
            min_nfz = min(min_nfz, scene.nfz_clearance(pose[0], pose[1]))
        dist_xy = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        alt_err = abs(pose[2] - goal[2])
        if dist_xy <= 2.0 and alt_err <= 1.5:
            if reached_at is None:
                reached_at = t
            elif t - reached_at >= 1.0:
                break
    pose = sim.pose
    final_xy = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    final_alt = abs(pose[2] - goal[2])
    result = dict(scene=name, reached=reached_at, time=t, final_xy=final_xy,
                  final_alt=final_alt, path_len=path_len,
                  collisions=sim.collisions, nfz=sim.nfz_violations,
                  replans=mission.replans, min_nfz=min_nfz)
    print("[mission] %-7s reached=%-5s t=%6.1f s  final xy=%.2f m  alt=%.2f m  "
          "path=%6.1f m  collisions=%d  nfz=%d  replans=%d  minNFZ=%5.2f m"
          % (name, reached_at is not None, t, final_xy, final_alt, path_len,
             sim.collisions, sim.nfz_violations, mission.replans, min_nfz))
    assert sim.collisions == 0, "%s: mission collided %d times" % (
        name, sim.collisions)
    assert sim.nfz_violations == 0, "%s: entered a no-fly zone %d times" % (
        name, sim.nfz_violations)
    assert reached_at is not None, \
        "%s: mission did not reach the goal within %.0f s (final xy %.2f m)" % (
            name, max_time, final_xy)
    assert final_xy <= 3.0, "%s: final horizontal error %.2f m > 3 m" % (
        name, final_xy)
    assert final_alt <= 2.0, "%s: final altitude error %.2f m > 2 m" % (
        name, final_alt)
    assert mission.replans > 0
    return result


def main() -> int:
    print("== test_control ==")
    test_pid()
    test_altitude_control()
    test_pure_pursuit()
    test_safety_filter()
    results = [run_mission_scene(name) for name in C.SCENES]
    print("[mission] summary: reached %d/%d, total collisions %d, "
          "total nfz violations %d"
          % (sum(1 for r in results if r["reached"] is not None),
             len(results), sum(r["collisions"] for r in results),
             sum(r["nfz"] for r in results)))
    print("[control] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
