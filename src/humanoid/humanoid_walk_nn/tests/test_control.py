#!/usr/bin/env python3
"""Control tests: PID / pure pursuit / safety filter / balance limits and
closed-loop walking missions in the hall, warehouse and obstacle scenes.

Run: ``/home/lxy/roscx_runtime/venv/bin/python tests/test_control.py``

Acceptance: ``MissionController`` walks a real ``HumanoidSim`` from
``scene.start`` to ``scene.goal`` (<= 0.6 m) with 0 collisions and 0 falls in
every scene.  Metrics printed per scene: time, final error, path length,
re-plans, obstacle stops, fall recoveries.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from humanoid_walk_nn import config as C
from humanoid_walk_nn.control import (MissionController, PID, PurePursuit,
                                      SafetyFilter, WalkingController)
from humanoid_walk_nn.scenes import get_scene
from humanoid_walk_nn.sim import HumanoidSim


def _beam_angles() -> np.ndarray:
    """Full-circle lidar beam angles, matching ``HumanoidSim.lidar_beam_angles``."""
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


def test_pure_pursuit() -> None:
    pp = PurePursuit(lookahead=0.8)
    path = [(2.0, 6.0), (10.0, 6.0)]
    v, omega = pp.compute((2.0, 6.0, 0.0, 0.0), path)
    assert v > 0.3 and abs(omega) < 0.05, "straight path tracking wrong"
    # facing +y while the path is to the right (+x) -> turn right (negative)
    v2, omega2 = pp.compute((2.0, 6.0, 0.0, math.pi / 2.0), path)
    assert omega2 < -0.3, "should turn right toward the +x path"
    # at the path end the robot must stop
    v3, omega3 = pp.compute((9.95, 6.0, 0.0, 0.0), path)
    assert v3 == 0.0, "path end should stop (v=%.3f)" % v3
    assert abs(omega3) < 0.1
    assert pp.compute((2.0, 6.0, 0.0, 0.0), []) == (0.0, 0.0)
    print("[pursuit] straight / turning / path-end OK (lookahead 0.8 m)")


def test_safety_filter() -> None:
    beams = _beam_angles()
    safety = SafetyFilter()
    clear = np.full(C.LIDAR_BEAMS, C.LIDAR_RANGE_MAX)
    assert safety.speed_scale(clear, beams) == 1.0
    center = C.LIDAR_BEAMS // 2
    mid = clear.copy()
    mid[center] = 0.45
    scale = safety.speed_scale(mid, beams)
    assert 0.0 < scale < 1.0, "0.45 m clearance should slow the robot"
    hard = clear.copy()
    hard[center] = 0.30
    assert safety.speed_scale(hard, beams) == 0.0, "0.30 m should stop walking"
    cmd = safety.apply([0.8, 0.2], hard, beams)
    assert cmd[0] == 0.0 and abs(cmd[1] - 0.2) < 1e-9, \
        "safety filter must only cut the walk speed"
    print("[safety] clear / slow (scale %.2f) / stop below 0.35 m OK" % scale)


def test_balance_limits() -> None:
    wc = WalkingController()
    # combined command over the balance budget: the turn is reduced first
    v, omega = wc.clamp_balance(0.90, 0.90, 0.0)
    assert abs(v * omega) <= wc.product_limit + 1e-9, \
        "|v*omega| %.3f exceeds the balance budget %.3f" % (
            abs(v * omega), wc.product_limit)
    assert v > 0.0
    # 70 % of the balance limit: both speed and turn are scaled down
    v_ok, _ = wc.clamp_balance(0.90, 0.20, 0.0)
    v_soft, _ = wc.clamp_balance(0.90, 0.20, 0.70 * C.BALANCE_LIMIT)
    assert v_soft < v_ok, "soft balance limit should slow the robot down"
    # beyond the limit the robot stands still
    assert wc.clamp_balance(0.90, 0.80, C.BALANCE_LIMIT) == (0.0, 0.0)
    assert wc.clamp_balance(0.90, 0.80, -1.2 * C.BALANCE_LIMIT) == (0.0, 0.0)
    # a straight path never violates the budget even when turning
    path = [(2.0, 6.0), (10.0, 6.0), (10.0, 1.5)]
    v_c, w_c = wc.compute((2.0, 6.0, 0.0, 0.0), path,
                          np.full(C.LIDAR_BEAMS, C.LIDAR_RANGE_MAX),
                          _beam_angles(), balance=0.0)
    assert abs(v_c * w_c) <= wc.product_limit + 1e-9
    print("[balance] product clamp / soft slowdown / hard stop OK "
          "(budget %.2f m*rad/s)" % wc.product_limit)


def test_fall_recovery() -> None:
    scene = get_scene("hall")
    sim = HumanoidSim("hall", seed=11, noise=True)
    mission = MissionController(scene)
    goal = np.array(scene.goal, dtype=np.float64)
    scan = sim.lidar()
    angles = sim.lidar_beam_angles()
    cmd = mission.update(sim.pose, goal, scan, angles, 0.05, balance=0.0,
                         falls=1)
    assert np.allclose(cmd, 0.0), \
        "after a fall the controller must pause (got %s)" % cmd
    assert mission.recoveries == 1
    for _ in range(10):          # 0.5 s of sim time: still inside the 0.8 s window
        cmd = mission.update(sim.pose, goal, scan, angles, 0.05,
                             balance=0.0, falls=1)
        assert np.allclose(cmd, 0.0), "still recovering after 0.5 s of sim time"
    resumed = False
    for _ in range(15):          # past the recovery window the mission resumes
        cmd = mission.update(sim.pose, goal, scan, angles, 0.05,
                             balance=0.0, falls=1)
        if not np.allclose(cmd, 0.0):
            resumed = True
            break
    assert resumed, "mission did not resume after the fall-recovery window"
    print("[recovery] fall detected -> commanded standstill for 0.8 s OK")


def run_mission_scene(name: str, max_time: float = 150.0) -> dict:
    """跑一条 scene 的完整任务；辅助函数（无 test_ 前缀，避免被 pytest 收集）。"""
    scene = get_scene(name)
    sim = HumanoidSim(name, seed=C.DEFAULT_SEED, noise=True)
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
    while t < max_time:
        if k % 4 == 0:
            scan = sim.lidar()
            angles = sim.lidar_beam_angles()
        k += 1
        cmd = mission.update(sim.pose, goal, scan, angles, dt,
                             balance=sim.balance, falls=sim.falls)
        sim.step(cmd, dt)
        t += dt
        pose = sim.pose
        path_len += math.hypot(pose[0] - prev[0], pose[1] - prev[1])
        prev = pose.copy()
        dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        if dist <= 0.60 and abs(sim.speed) < 0.05:
            if reached_at is None:
                reached_at = t
            elif t - reached_at >= 0.5:
                break
        else:
            reached_at = None
    pose = sim.pose
    final_dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    result = dict(scene=name, reached=reached_at, time=t, final=final_dist,
                  path_len=path_len, replans=mission.replans,
                  obstacles=mission.stopped_for_obstacle,
                  carts=mission.stopped_for_cart,
                  recoveries=mission.recoveries,
                  collisions=sim.collisions, falls=sim.falls,
                  steps=sim.steps)
    print("[mission] %-10s reached=%-5s t=%6.1f s  final=%.2f m  path=%6.1f m  "
          "replans=%d  obstacle_stops=%d  cart_stops=%d  recoveries=%d  "
          "collisions=%d  falls=%d  steps=%d"
          % (name, reached_at is not None, t, final_dist, path_len,
             mission.replans, mission.stopped_for_obstacle,
             mission.stopped_for_cart, mission.recoveries, sim.collisions,
             sim.falls, sim.steps))
    assert sim.collisions == 0, "%s: mission collided %d times" % (
        name, sim.collisions)
    assert sim.falls == 0, "%s: humanoid fell %d times" % (name, sim.falls)
    assert reached_at is not None, \
        "%s: mission did not reach the goal within %.0f s (final %.2f m)" % (
            name, max_time, final_dist)
    assert final_dist <= 0.60, "%s: final error %.2f m > 0.6 m" % (
        name, final_dist)
    assert mission.replans > 0
    return result


def main() -> int:
    print("== test_control ==")
    test_pid()
    test_pure_pursuit()
    test_safety_filter()
    test_balance_limits()
    test_fall_recovery()
    results = [run_mission_scene(name) for name in C.SCENES]
    print("[mission] summary: reached %d/%d, collisions %d, falls %d, "
          "obstacle stops %d, cart stops %d, recoveries %d"
          % (sum(1 for r in results if r["reached"] is not None), len(results),
             sum(r["collisions"] for r in results),
             sum(r["falls"] for r in results),
             sum(r["obstacles"] for r in results),
             sum(r["carts"] for r in results),
             sum(r["recoveries"] for r in results)))
    print("[control] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
