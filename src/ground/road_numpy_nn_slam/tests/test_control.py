#!/usr/bin/env python3
"""Control tests: PID / speed control / Stanley / light+pedestrian rules and
closed-loop missions in the downtown, campus and construction road scenes.

Run: ``/home/lxy/roscx_runtime/venv/bin/python tests/test_control.py``

Acceptance: ``MissionController`` drives a real ``RoadSim`` from ``scene.start``
to ``scene.goal`` (<= 3 m) with 0 collisions, 0 red-light violations and
0 pedestrians hit, in every scene.  Metrics printed per scene: time, final
error, path length, re-plans, light stops, pedestrian stops, off-road time.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from road_numpy_nn_slam import config as C
from road_numpy_nn_slam.control import (MissionController, PID, SafetyFilter,
                                 SpeedController, Stanley)
from road_numpy_nn_slam.scenes import get_scene
from road_numpy_nn_slam.sim import RoadSim


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


def test_speed_control() -> None:
    sim = RoadSim("downtown", seed=11, noise=True)
    ctrl = SpeedController()
    dt = 0.05
    t = 0.0
    max_speed = 0.0
    stopped_at = None
    for _ in range(int(14.0 / dt)):
        target = 6.0 if t < 5.0 else 0.0
        throttle = ctrl.throttle(target, sim.speed, dt)
        sim.step([throttle, 0.0], dt)
        t += dt
        max_speed = max(max_speed, sim.speed)
        if t > 5.0 and stopped_at is None and sim.speed < 0.2:
            stopped_at = t
        if stopped_at is not None and t > stopped_at + 1.5:
            break
    print("[speed] target 6 m/s reached %.2f m/s, stopped at %.2f s, "
          "final %.2f m/s" % (max_speed, stopped_at if stopped_at else -1.0,
                              sim.speed))
    assert max_speed > 5.0, "speed controller too slow (%.2f m/s)" % max_speed
    assert stopped_at is not None, "speed controller failed to stop"
    assert sim.speed < 0.3, "car still rolling at %.2f m/s" % sim.speed


def test_stanley() -> None:
    stanley = Stanley()
    path = [(0.0, 0.0), (100.0, 0.0)]
    angle, cross, head = stanley.compute((0.0, 0.0, 0.0), path, speed=8.0)
    assert abs(angle) < 0.02 and abs(cross) < 0.02 and abs(head) < 1e-6, \
        "on-path car should steer straight"
    # car left of the path -> the path is to its right -> steer right (negative)
    angle2, cross2, _ = stanley.compute((0.0, 1.0, 0.0), path, speed=8.0)
    assert cross2 < -0.9, "cross-track sign wrong: %.2f" % cross2
    assert angle2 < -0.05, "should steer right toward the path: %.2f" % angle2
    # car right of the path -> steer left (positive)
    angle3, cross3, _ = stanley.compute((0.0, -1.0, 0.0), path, speed=8.0)
    assert cross3 > 0.9 and angle3 > 0.05, "cross-track/steer sign wrong"
    # nose pointing left of the path tangent -> steer right
    angle4, _, head4 = stanley.compute((0.0, 0.0, 0.2), path, speed=8.0)
    assert head4 < -0.1 and angle4 < 0.0, "heading-error sign wrong"
    assert abs(angle4) <= C.MAX_STEER + 1e-9
    print("[stanley] straight / cross-track / heading-error signs OK "
          "(cross %.2f m -> steer %.3f rad)" % (cross2, angle2))


def test_light_and_pedestrian_rules() -> None:
    grid = np.zeros((40, 40), dtype=bool)     # 10 x 10 m of free grid
    mission = MissionController(grid, resolution=0.25)
    assert mission._light_limit(None) == (None, False)
    assert mission._light_limit({"state": "green", "distance": 20.0}) == \
        (None, False)
    # red light inside the 25 m reaction window: smooth speed reduction
    v_red, stop_red = mission._light_limit({"state": "red", "distance": 20.0})
    assert v_red is not None and 0.0 < v_red < C.MAX_SPEED
    assert not stop_red, "should still be rolling 20 m before the light"
    # red light at the stop line: come to a full stop
    v_stop, stop_now = mission._light_limit({"state": "red", "distance": 6.0})
    assert v_stop == 0.0 and stop_now, "should stop before the red light"
    # too far away (stop line > 25 m): no reaction yet
    assert mission._light_limit({"state": "red", "distance": 60.0}) == \
        (None, False)
    # yellow light that can still be stopped for is treated as a stop
    v_yellow, _ = mission._light_limit({"state": "yellow", "distance": 20.0})
    assert v_yellow is not None
    # ... but a yellow light too close to stop for is legal to cross
    mission._speed_est = 8.0
    assert mission._light_limit({"state": "yellow", "distance": 7.0}) == \
        (None, False)

    pose = np.array([0.0, 0.0, 0.0, 0.0])
    assert mission._ped_limit(None, pose) == (None, False)
    v_far, stop_far = mission._ped_limit([(6.0, 0.2)], pose)
    assert v_far is not None and not stop_far
    v_close, stop_close = mission._ped_limit([(1.0, 0.2)], pose)
    assert v_close == 0.0 and stop_close, "should stop for a pedestrian ahead"
    v_caution, stop_caution = mission._ped_limit([(5.0, 8.0)], pose)
    assert v_caution == 4.5 and not stop_caution, \
        "pedestrian near the road should slow the car down"
    assert mission._ped_limit([(5.0, 13.0)], pose) == (None, False), \
        "pedestrians far to the side must not slow the car"
    assert mission._ped_limit([(-5.0, 0.0)], pose) == (None, False), \
        "pedestrians behind the car must not stop it"
    safety = SafetyFilter()
    clear = np.full(C.LIDAR_BEAMS, C.LIDAR_RANGE_MAX)
    beams = np.linspace(-math.pi, math.pi, C.LIDAR_BEAMS, endpoint=False)
    assert safety.speed_limit(clear, beams) >= C.MAX_SPEED - 1e-9
    print("[rules] red/yellow-light window, stop line, pedestrian sector OK")


def test_light_stop_closed_loop() -> None:
    """Drive at a red/yellow light: the car must wait behind the stop line."""
    scene = get_scene("downtown")
    sim = RoadSim("downtown", seed=C.DEFAULT_SEED, noise=True)
    sim.reset(pose=(45.0, 70.0, 0.0, 0.0))
    mission = MissionController(scene)
    goal = np.array([130.0, 70.0, 0.0])
    dt = 0.05
    scan = sim.lidar()
    angles = sim.lidar_beam_angles()
    t = 0.0
    k = 0
    max_x = sim.x
    while t < 30.0:
        if k % 4 == 0:
            scan = sim.lidar()
            angles = sim.lidar_beam_angles()
        k += 1
        cmd = mission.update(sim.pose, goal, scan, angles, dt,
                             light=sim.traffic_light(),
                             pedestrians=scene.pedestrian_positions(sim.time))
        sim.step(cmd, dt)
        t += dt
        max_x = max(max_x, sim.x)
        if max_x > 75.0:
            break
    print("[light-stop] waited %d time(s) at the light, max x=%.1f m "
          "(violation line x=65), red-light violations=%d, collisions=%d, "
          "t=%.1f s"
          % (mission.stopped_for_light, max_x, sim.red_light_violations,
             sim.collisions, t))
    assert mission.stopped_for_light >= 1, \
        "should wait for the red/yellow light at the stop line"
    assert sim.red_light_violations == 0, "must not cross on red"
    assert sim.collisions == 0
    assert max_x > 72.0, "should continue after the light turns green"


def test_pedestrian_stop_closed_loop() -> None:
    """Approach a pedestrian crossing: the car must stop and never hit them."""
    scene = get_scene("downtown")
    sim = RoadSim("downtown", seed=C.DEFAULT_SEED, noise=True)
    sim.reset(pose=(75.0, 70.0, 0.0, 0.0))
    mission = MissionController(scene)
    goal = np.array([130.0, 70.0, 0.0])
    dt = 0.05
    scan = sim.lidar()
    angles = sim.lidar_beam_angles()
    t = 0.0
    k = 0
    max_x = sim.x
    while t < 25.0:
        if k % 4 == 0:
            scan = sim.lidar()
            angles = sim.lidar_beam_angles()
        k += 1
        cmd = mission.update(sim.pose, goal, scan, angles, dt,
                             light=sim.traffic_light(),
                             pedestrians=scene.pedestrian_positions(sim.time))
        sim.step(cmd, dt)
        t += dt
        max_x = max(max_x, sim.x)
        if max_x > 95.0:
            break
    print("[ped-stop] pedestrian stops=%d, pedestrian yields=%d, max x=%.1f m "
          "(crossing x=88), collisions=%d, pedestrians hit=%d, t=%.1f s"
          % (mission.stopped_for_pedestrian, mission.yields_for_pedestrian,
             max_x, sim.collisions, sim.pedestrians_hit, t))
    assert mission.stopped_for_pedestrian >= 1, \
        "should stop for the pedestrian on the crossing"
    assert sim.pedestrians_hit == 0, "must not hit a pedestrian"
    assert sim.collisions == 0


def run_mission_scene(name: str, max_time: float = 180.0) -> dict:
    scene = get_scene(name)
    sim = RoadSim(name, seed=C.DEFAULT_SEED, noise=True)
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
        light = sim.traffic_light()
        pedestrians = scene.pedestrian_positions(sim.time)
        cmd = mission.update(sim.pose, goal, scan, angles, dt,
                             light=light, pedestrians=pedestrians)
        sim.step(cmd, dt)
        t += dt
        pose = sim.pose
        path_len += math.hypot(pose[0] - prev[0], pose[1] - prev[1])
        prev = pose.copy()
        dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        if dist <= 2.5 and sim.speed < 0.4:
            if reached_at is None:
                reached_at = t
            elif t - reached_at >= 0.5:
                break
    pose = sim.pose
    final_dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    result = dict(scene=name, reached=reached_at, time=t, final=final_dist,
                  path_len=path_len, replans=mission.replans,
                  light_stops=mission.stopped_for_light,
                  ped_stops=mission.stopped_for_pedestrian,
                  light_yields=mission.yields_for_light,
                  ped_yields=mission.yields_for_pedestrian,
                  collisions=sim.collisions,
                  red=sim.red_light_violations,
                  peds_hit=sim.pedestrians_hit,
                  off_road=sim.off_road_time)
    print("[mission] %-12s reached=%-5s t=%6.1f s  final=%.2f m  path=%6.1f m  "
          "replans=%d  light_stops=%d  light_yields=%d  ped_stops=%d  "
          "ped_yields=%d  collisions=%d  red=%d  peds_hit=%d  off_road=%.1f s"
          % (name, reached_at is not None, t, final_dist, path_len,
             mission.replans, mission.stopped_for_light,
             mission.yields_for_light, mission.stopped_for_pedestrian,
             mission.yields_for_pedestrian, sim.collisions,
             sim.red_light_violations, sim.pedestrians_hit,
             sim.off_road_time))
    assert sim.collisions == 0, "%s: mission collided %d times" % (
        name, sim.collisions)
    assert sim.red_light_violations == 0, \
        "%s: ran a red light %d times" % (name, sim.red_light_violations)
    assert sim.pedestrians_hit == 0, \
        "%s: hit %d pedestrians" % (name, sim.pedestrians_hit)
    assert reached_at is not None, \
        "%s: mission did not reach the goal within %.0f s (final %.2f m)" % (
            name, max_time, final_dist)
    assert final_dist <= 3.0, "%s: final error %.2f m > 3 m" % (
        name, final_dist)
    assert mission.replans > 0
    return result


def main() -> int:
    print("== test_control ==")
    test_pid()
    test_speed_control()
    test_stanley()
    test_light_and_pedestrian_rules()
    test_light_stop_closed_loop()
    test_pedestrian_stop_closed_loop()
    results = [run_mission_scene(name) for name in C.SCENES]
    print("[mission] summary: reached %d/%d, collisions %d, red-light %d, "
          "pedestrians hit %d, light stops %d, pedestrian stops %d"
          % (sum(1 for r in results if r["reached"] is not None), len(results),
             sum(r["collisions"] for r in results),
             sum(r["red"] for r in results),
             sum(r["peds_hit"] for r in results),
             sum(r["light_stops"] for r in results),
             sum(r["ped_stops"] for r in results)))
    print("[control] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
