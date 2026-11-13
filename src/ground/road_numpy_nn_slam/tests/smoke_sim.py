#!/usr/bin/env python3
"""Smoke test: bicycle dynamics, lidar, camera, traffic lights and collisions."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from road_numpy_nn_slam.sim import RoadSim


def main() -> int:
    sim = RoadSim("downtown", seed=1)
    print("start pose", np.round(sim.pose, 2), "speed", sim.speed)
    t0 = time.time()
    for _ in range(100):
        sim.step([0.6, 0.0], 0.05)
    print("accel 5s -> v %.2f m/s, moved %.2f m, yaw %.2f (%.0f steps/s)"
          % (sim.v, np.hypot(sim.x - sim.scene.start[0], sim.y - sim.scene.start[1]),
             sim.yaw, 100 / (time.time() - t0)))
    # steering
    sim.reset()
    for _ in range(40):
        sim.step([0.5, 0.6], 0.05)
    print("steer right 2s -> yaw %.2f rad, v %.2f" % (sim.yaw, sim.v))
    # brake
    sim.reset()
    for _ in range(60):
        sim.step([0.8, 0.0], 0.05)
    v_before = sim.v
    for _ in range(30):
        sim.step([-0.8, 0.0], 0.05)
    print("brake 1.5s -> v %.2f -> %.2f" % (v_before, sim.v))
    # sensors
    scan = sim.lidar()
    print("lidar", scan.shape, "min %.2f max %.2f mean %.2f"
          % (scan.min(), scan.max(), scan.mean()))
    cam = sim.camera()
    print("camera", cam.shape, cam.dtype, "mean", int(cam.mean()))
    print("gnss", np.round(sim.gnss(), 2), "imu", np.round(sim.imu(), 3))
    # traffic lights
    sim.reset((20.0, 70.0, 0.0, 0.0))
    states = set()
    for k in range(400):
        sim.step([0.5, 0.0], 0.05)
        lt = sim.traffic_light()
        if lt:
            states.add(lt["state"])
        if sim.x > 45:
            break
    print("lights seen:", states, "x=%.1f red_violations=%d"
          % (sim.x, sim.red_light_violations))
    # collision with a parked car
    sim.reset((18.0, 70.0, 0.0, 0.0))
    sim.x, sim.y = 18.0, 32.2
    for _ in range(120):
        sim.step([0.8, 0.0], 0.05)
    print("drive into parked car -> collisions", sim.collisions, "v", round(sim.v, 2))
    # off-road slows down
    sim.reset((10.0, 20.0, 0.0, 0.0))
    for _ in range(60):
        sim.step([1.0, 0.0], 0.05)
    print("off-road 3s -> v %.2f, off_road_time %.1f" % (sim.v, sim.off_road_time))
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
