#!/usr/bin/env python3
"""Smoke test for the musculoskeletal arm simulator."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from msk_arm_nn.sim import MskArmSim


def main() -> int:
    sim = MskArmSim("reach", seed=1)
    print("start q(deg)", np.round(np.degrees(sim.q), 1), "ee", np.round(sim.ee(), 3))
    t0 = time.time()
    for _ in range(50):
        sim.step([0.35, 0.0, 0.25, 0.0], 0.02)     # lift the arm a little
    print("after 1s flexors -> q(deg)", np.round(np.degrees(sim.q), 1),
          "ee", np.round(sim.ee(), 3), "(%.0f steps/s)" % (50 / (time.time() - t0)))
    print("forces", np.round(sim.muscle_forces(), 1), "activations",
          np.round(sim.muscle_activations(), 2))
    # hold with co-contraction
    sim.reset()
    for _ in range(150):
        sim.step([0.20, 0.20, 0.12, 0.12], 0.02)
    print("co-contraction 3s -> q(deg)", np.round(np.degrees(sim.q), 1),
          "collisions", sim.collisions)
    # IK round trip
    target = sim.scene.targets[0]
    q = sim.ik(*target)
    print("IK for target", target, "->", None if q is None else np.round(np.degrees(q), 1))
    # depth camera
    depth = sim.depth_camera()
    print("depth", depth.shape, depth.dtype, "mean %.2f max %.2f" % (depth.mean(), depth.max()))
    # shelf collision when driving straight into the obstacle
    sim = MskArmSim("shelf", seed=2)
    for _ in range(200):
        sim.step([0.55, 0.0, 0.35, 0.0], 0.02)
    print("shelf push 4s -> ee", np.round(sim.ee(), 3), "collisions", sim.collisions,
          "q(deg)", np.round(np.degrees(sim.q), 1))
    # target reach with simple P controller (EE -> target via joint P)
    sim = MskArmSim("reach", seed=3)
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
