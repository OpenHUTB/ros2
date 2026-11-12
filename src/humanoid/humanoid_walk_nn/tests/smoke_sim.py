#!/usr/bin/env python3
"""Smoke test for the simplified humanoid walking simulator."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from humanoid_walk_nn.sim import HumanoidSim


def main() -> int:
    sim = HumanoidSim("hall", seed=1)
    print("start", np.round(sim.pose, 2))
    t0 = time.time()
    for _ in range(100):
        sim.step([1.0, 0.0], 0.05)
    print("walk 5s -> v %.2f m/s, moved %.2f m, steps %d, falls %d, balance %.3f (%.0f steps/s)"
          % (sim.v, np.hypot(sim.x - sim.scene.start[0], sim.y - sim.scene.start[1]),
             sim.steps, sim.falls, sim.balance, 100 / (time.time() - t0)))
    # turning at speed should not immediately fall
    sim.reset()
    for _ in range(80):
        sim.step([0.8, 1.0], 0.05)
    print("turn 4s -> yaw %.2f, v %.2f, balance %.3f, falls %d"
          % (sim.yaw, sim.v, sim.balance, sim.falls))
    # sharp continuous turn should eventually lose balance
    sim.reset()
    for _ in range(200):
        sim.step([1.2, 1.5], 0.05)
    print("sharp turn 10s -> falls %d, balance %.3f" % (sim.falls, sim.balance))
    # sensors
    scan = sim.lidar()
    print("lidar", scan.shape, "min %.2f max %.2f mean %.2f"
          % (scan.min(), scan.max(), scan.mean()))
    depth = sim.camera_depth()
    print("depth", depth.shape, depth.dtype, "min %.2f max %.2f mean %.2f"
          % (depth.min(), depth.max(), depth.mean()))
    print("imu", np.round(sim.imu(), 3), "joints", np.round(sim.joints(), 3))
    # wall collision
    sim.reset((1.0, 6.0, 0.0, 3.14159))
    for _ in range(80):
        sim.step([1.0, 0.0], 0.05)
    print("walk into wall -> collisions %d, v %.2f" % (sim.collisions, sim.v))
    # cart obstacle in warehouse
    simw = HumanoidSim("warehouse", seed=2)
    for _ in range(200):
        simw.step([0.9, 0.0], 0.05)
    print("warehouse walk 10s -> pose", np.round(simw.pose, 2),
          "collisions", simw.collisions, "falls", simw.falls)
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
