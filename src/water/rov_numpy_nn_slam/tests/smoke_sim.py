#!/usr/bin/env python3
"""Smoke test: physics, sensors and timing of the ROV simulation."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from rov_numpy_nn_slam.sim import ROVSim


def main() -> int:
    sim = ROVSim("pool", seed=1)
    print("start pose", np.round(sim.pose, 3), "depth", round(sim.depth(), 3))
    # forward acceleration / top speed
    t0 = time.time()
    n = 100
    for _ in range(n):
        sim.step([1.2, 0.0, 0.0, 0.0], 0.02)
    dt = time.time() - t0
    pose = sim.pose
    print("surge 2s -> moved %.2f m, yaw %.3f, depth %.2f, %.0f steps/s"
          % (np.hypot(pose[0] - sim.scene.start[0], pose[1] - sim.scene.start[1]),
             pose[3], sim.depth(), n / dt))
    # depth control
    sim.reset()
    for _ in range(100):
        sim.step([0.0, 0.0, 0.5, 0.0], 0.02)
    print("heave to depth cmd 0.5 -> depth %.2f (start %.2f)" % (sim.depth(), sim.scene.start[2]))
    # yaw
    sim.reset()
    for _ in range(50):
        sim.step([0.0, 0.0, 0.0, 1.0], 0.02)
    print("yaw rate cmd -> yaw %.2f rad after 1 s" % sim.pose[3])
    # sensors
    scan = sim.sonar()
    print("sonar", scan.shape, "min %.2f max %.2f mean %.2f"
          % (scan.min(), scan.max(), scan.mean()))
    cam = sim.camera()
    print("camera", cam.shape, cam.dtype, "mean", int(cam.mean()))
    print("dvl", np.round(sim.dvl(), 3), "imu", np.round(sim.imu(), 3))
    # collision test: drive into the wall
    sim.reset()
    for _ in range(300):
        sim.step([1.2, 0.0, 0.0, 0.0], 0.02)
    print("drive->wall collisions", sim.collisions)
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
