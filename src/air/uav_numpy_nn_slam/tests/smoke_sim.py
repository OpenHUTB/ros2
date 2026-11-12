#!/usr/bin/env python3
"""Smoke test: quadrotor physics, sensors, wind and timing."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from uav_numpy_nn_slam.sim import UAVSim


def main() -> int:
    sim = UAVSim("city", seed=1)
    print("start pose", np.round(sim.pose, 2))
    # hover check
    p0 = sim.pose.copy()
    t0 = time.time()
    n = 100
    for _ in range(n):
        sim.step([0.0, 0.0, 0.0, 0.0], 0.05)
    dt = time.time() - t0
    p1 = sim.pose
    print("hover 5s -> drift %.2f m, dz %.2f m, %.0f steps/s"
          % (np.hypot(p1[0] - p0[0], p1[1] - p0[1]), p1[2] - p0[2], n / dt))
    # forward flight
    sim.reset()
    for _ in range(80):
        sim.step([6.0, 0.0, 0.0, 0.0], 0.05)
    p = sim.pose
    print("forward 4s -> moved %.2f m, alt %.2f, yaw %.2f, collisions %d"
          % (np.hypot(p[0] - sim.scene.start[0], p[1] - sim.scene.start[1]),
             p[2], p[3], sim.collisions))
    # yaw
    sim.reset()
    for _ in range(20):
        sim.step([0.0, 0.0, 0.0, 1.0], 0.05)
    print("yaw cmd 1.0 -> yaw %.2f rad after 1s" % sim.pose[3])
    # climb
    sim.reset()
    for _ in range(20):
        sim.step([0.0, 0.0, 2.0, 0.0], 0.05)
    print("climb cmd 2 m/s -> alt %.2f (start %.2f)" % (sim.pose[2], sim.scene.start[2]))
    # sensors
    scan = sim.lidar()
    print("lidar", scan.shape, "min %.2f max %.2f mean %.2f"
          % (scan.min(), scan.max(), scan.mean()))
    cam = sim.camera()
    print("camera", cam.shape, cam.dtype, "mean", int(cam.mean()))
    print("baro", round(sim.altitude(), 2), "gps", np.round(sim.gps(), 2),
          "imu", np.round(sim.imu(), 3), "nfz clearance", round(sim.nfz_clearance(), 2))
    # collision: fly into a building
    sim.reset((60.0, 40.0, 32.0, 0.0))
    for _ in range(160):
        sim.step([8.0, 0.0, 0.0, 0.0], 0.05)
    print("fly->building collisions", sim.collisions, "pose", np.round(sim.pose, 1))
    # NFZ entry
    sim.reset((40.0, 65.0, 32.0, 0.0))
    for _ in range(80):
        sim.step([6.0, 0.0, 0.0, 0.0], 0.05)
    print("fly->nfz violations", sim.nfz_violations, "inside", sim.scene.inside_nfz(*sim.pose[:2]))
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
