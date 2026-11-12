#!/usr/bin/env python3
"""Quick diagnostic of the end-to-end policy (not part of the deliverable)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ca_drone.models.e2e import E2ENet, goal_features
from ca_drone.nn.autograd import Tensor
from ca_drone.pipeline import DEPTH_H, DEPTH_W, FOV, MAX_RANGE
from ca_drone.render.scene import depth_image
from ca_drone.sim import make_world

model, _ = E2ENet.load("weights/e2e.npz")

for scene, pos, yaw in [
    ("sparse", (2.0, 0.0), 0.0),
    ("corridor", (6.0, 0.0), 0.0),
    ("corridor", (12.0, 0.0), 0.0),
    ("dense", (20.0, 0.0), 0.0),
]:
    w = make_world(scene)
    p = np.array([pos[0], pos[1], 3.0])
    depth, _ = depth_image(w, p, yaw, DEPTH_W, DEPTH_H, FOV, MAX_RANGE)
    delta = w.goal[:2] - p[:2]
    c, s = np.cos(yaw), np.sin(yaw)
    gb = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
    feat = goal_features(gb, float(np.linalg.norm(delta)))
    out = model(Tensor((depth / MAX_RANGE)[None, None]), Tensor(feat[None])).data[0]
    print("%-8s pos=(%.1f,%.1f) depth min=%.2f mean=%.2f gb=(%.1f,%.1f) -> cmd=(%.2f,%.2f)" % (
        scene, pos[0], pos[1], depth.min(), depth.mean(), gb[0], gb[1], out[0], out[1]))
