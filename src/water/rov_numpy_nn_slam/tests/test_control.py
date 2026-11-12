#!/usr/bin/env python3
"""控制模块测试：PID / 定深 / 纯跟踪 / 安全滤波 + pool 场景任务级集成。

运行：``/home/lxy/roscx_runtime/venv/bin/python tests/test_control.py``
集成验收：MissionController 驱动真实 ROVSim 从 scene.start 到 goal 0.8 m 内，
≤ 90 s、0 碰撞、定深误差 ≤ 0.5 m。
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.control import (DepthController, MissionController, PID,
                                PurePursuit, SafetyFilter)
from rov_numpy_nn_slam.scenes import get_scene
from rov_numpy_nn_slam.sim import ROVSim


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


def test_depth_control() -> None:
    sim = ROVSim("pool", seed=11, noise=True)
    depth = DepthController(target=2.6)
    dt = 0.02
    for _ in range(int(12.0 / dt)):
        heave = depth.update(sim.depth(), 2.6, dt)
        sim.step([0.0, 0.0, heave, 0.0], dt)
    err = abs(sim.pose[2] - 2.6)
    print("[depth] target 2.60 m -> %.3f m, error %.3f m, collisions %d"
          % (sim.pose[2], err, sim.collisions))
    assert err < 0.3, "depth hold error %.3f m too large" % err
    assert sim.collisions == 0


def test_pure_pursuit() -> None:
    pp = PurePursuit(lookahead=0.9)
    path = [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0)]
    vx, wz = pp.compute((0.0, 0.0, 0.0), path)
    assert vx > 0.5 and abs(wz) < 0.05, "straight path tracking wrong"
    vx2, wz2 = pp.compute((0.0, 0.0, math.pi / 2.0), path)
    assert vx2 > 0.0 and wz2 < -0.3, "should yaw right toward +x path"
    vx3, wz3 = pp.compute((6.0, 0.0, 0.0), path)
    assert vx3 == 0.0 and wz3 == 0.0, "path end should stop"
    assert pp.compute((0.0, 0.0, 0.0), []) == (0.0, 0.0)
    print("[pursuit] straight / turning / path-end behaviour OK")


def test_safety_filter() -> None:
    beams = np.linspace(-C.SONAR_FOV / 2, C.SONAR_FOV / 2, C.SONAR_BEAMS)
    safety = SafetyFilter(min_clearance=0.9)
    clear = np.full(C.SONAR_BEAMS, C.SONAR_RANGE_MAX)
    cmd = safety.apply([1.0, 0.0, 0.0, 0.2], clear, beams)
    assert abs(cmd[0] - 1.0) < 1e-9, "clear water should not change surge"
    hard = clear.copy()
    hard[C.SONAR_BEAMS // 2] = 0.40
    cmd = safety.apply([1.0, 0.0, 0.0, 0.2], hard, beams)
    assert cmd[0] == 0.0, "close obstacle should stop surge"
    soft = clear.copy()
    soft[C.SONAR_BEAMS // 2] = 0.75
    cmd = safety.apply([1.0, 0.0, 0.0, 0.2], soft, beams)
    assert 0.0 < cmd[0] < 1.0, "medium clearance should reduce surge"
    print("[safety] clear / slow / stop OK (front clearance %.2f m -> vx %.2f)"
          % (0.75, cmd[0]))


def test_mission_integration() -> int:
    scene = get_scene("pool")
    sim = ROVSim("pool", seed=C.DEFAULT_SEED, noise=True)
    mission = MissionController(scene)
    goal = np.array(scene.goal, dtype=np.float64)
    dt = 0.02
    max_time = 90.0
    t = 0.0
    reached_at = None
    min_clearance = C.SONAR_RANGE_MAX
    scan = sim.sonar()
    angles = sim.sonar_beam_angles()
    step_count = 0
    while t < max_time:
        if step_count % 5 == 0:
            scan = sim.sonar()
            angles = sim.sonar_beam_angles()
        step_count += 1
        cmd = mission.update(sim.pose, goal, scan, angles, dt)
        min_clearance = min(min_clearance, mission.safety.frontal_clearance(scan, angles))
        sim.step(cmd, dt)
        t += dt
        pose = sim.pose
        if math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= 0.60:
            reached_at = t
            break
    pose = sim.pose
    final_dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    depth_err = abs(pose[2] - goal[2])
    print("[mission] reached=%s after %.2f s, final dist %.2f m, "
          "depth err %.2f m, collisions %d, replans %d, min front clearance %.2f m"
          % (reached_at is not None, t, final_dist, depth_err, sim.collisions,
             mission.replans, min_clearance))
    assert sim.collisions == 0, "mission run collided %d times" % sim.collisions
    assert reached_at is not None, \
        "mission did not reach goal within %.0f s (final dist %.2f m)" % (max_time, final_dist)
    assert final_dist <= 0.8, "final distance %.2f m exceeds 0.8 m" % final_dist
    assert depth_err <= 0.5, "depth error %.2f m too large" % depth_err
    return 0


def main() -> int:
    print("== test_control ==")
    test_pid()
    test_depth_control()
    test_pure_pursuit()
    test_safety_filter()
    test_mission_integration()
    print("[control] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
