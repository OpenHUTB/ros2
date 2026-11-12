#!/usr/bin/env python3
"""Render reproducible demo GIFs for the musculoskeletal arm documentation."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from msk_arm_nn import config as C
from msk_arm_nn.render import ArmRenderer, frames_to_gif
from msk_arm_nn.scenes import get_scene
from msk_arm_nn.sim import MskArmSim


def _hud(sim, extra=None):
    ee = sim.ee()
    goal = sim.goal()
    err = math.hypot(ee[0] - goal[0], ee[1] - goal[1])
    d = {
        "q1": "%+.0f deg" % math.degrees(float(sim.q[0])),
        "q2": "%+.0f deg" % math.degrees(float(sim.q[1])),
        "ee": "(%.2f, %.2f)" % (ee[0], ee[1]),
        "reached": "%d/%d" % (sim.reached, len(sim.scene.targets)),
        "collisions": "%d" % sim.collisions,
        "time": "%.1f s" % sim.time,
    }
    if extra:
        d.update(extra)
    return d


# Muscle activation timeline for the teleoperation clip: lift the arm, hold,
# extend the elbow, relax (all activations stay gentle, no collision).
TELEOP_TIMELINE = (
    ("lift", 1.2), ("hold", 0.9), ("extend", 1.2), ("hold", 0.8),
    ("lift", 0.8), ("relax", 1.0), ("extend", 0.9), ("relax", 0.8),
)


def timeline_cmd(t: float):
    acc = 0.0
    for key, seconds in TELEOP_TIMELINE:
        acc += seconds
        if t < acc:
            if key == "lift":
                return [0.486, 0.150, 0.105, 0.030], key
            if key == "elbow":
                return [0.480, 0.150, 0.120, 0.040], key
            return [0.480, 0.150, 0.100, 0.030], "hold"
    return [0.10, 0.10, 0.10, 0.10], "-"


KEY_TEXT = {
    "lift": "LIFT  shoulder flexor", "extend": "EXTEND  elbow extensor",
    "hold": "HOLD  co-contraction", "relax": "RELAX  low activation",
}


def record_teleop(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    sim = MskArmSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = ArmRenderer(sim.scene)
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    for k in range(steps):
        t = k / C.CONTROL_HZ
        cmd, key = timeline_cmd(t)
        sim.step(cmd)
        q = sim.q.copy()
        trail.append((float(sim.ee()[0]), float(sim.ee()[1])))
        if k % capture_every == 0:
            frames.append(renderer.render(
                q, activations=sim.muscle_activations(), goal=sim.goal(),
                trail=trail, hud=_hud(sim), mode="TELEOP  muscles",
                key_label=key.upper(), message=KEY_TEXT.get(key, ""),
                time=sim.time, torque=sim.max_torque, reached=sim.reached,
                collisions=sim.collisions,
                ee_error=math.hypot(sim.ee()[0] - sim.goal()[0],
                                    sim.ee()[1] - sim.goal()[1])))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps)
    frames[len(frames) // 2].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "teleop", "scene": scene, "frames": len(frames),
            "bytes": size, "seconds": seconds, "collisions": sim.collisions,
            "path": out}


def record_reach(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    from msk_arm_nn.control import MuscleController
    from msk_arm_nn.slam import SlamFrontend

    sim = MskArmSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = ArmRenderer(sim.scene)
    slam = SlamFrontend(sim.scene)
    ctrl = MuscleController(sim.scene)
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    pose = sim.pose()
    for k in range(steps):
        depth = sim.depth_camera()
        pose = sim.pose()
        slam.update(depth)
        goal = sim.goal()
        try:
            cmd = ctrl.update(pose, goal, None, None, 1.0 / C.CONTROL_HZ)
        except TypeError:
            cmd = ctrl.update(pose, goal)
        sim.step(cmd)
        trail.append((float(pose[0]), float(pose[1])))
        if sim.success:
            break
        if k % capture_every == 0:
            frames.append(renderer.render(
                sim.q, activations=sim.muscle_activations(),
                path=getattr(ctrl, "path", None), goal=goal, trail=trail,
                occupancy=slam.occupancy() if hasattr(slam, "occupancy") else None,
                hud=_hud(sim, {"target": "%d/%d" % (sim.target_index + 1,
                                                    len(sim.scene.targets))}),
                mode="MUSCLE  control", key_label="AUTO",
                message="深度建图 + A* 规划 + IK + 重力补偿肌肉控制",
                coverage=float(slam.occupancy().mean()) if hasattr(slam, "occupancy") else 0.0,
                time=sim.time, torque=sim.max_torque, reached=sim.reached,
                collisions=sim.collisions,
                ee_error=math.hypot(pose[0] - goal[0], pose[1] - goal[1])))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps)
    frames[len(frames) // 3].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "reach", "scene": scene, "frames": len(frames),
            "bytes": size, "success": sim.success, "reached": sim.reached,
            "collisions": sim.collisions, "final_error_m": float(
                math.hypot(pose[0] - sim.goal()[0], pose[1] - sim.goal()[1])),
            "path": out}


def record_e2e(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    from msk_arm_nn.nn.cnn import CNN

    model = CNN.load(os.path.join(C.MODELS_DIR, "e2e_cnn.npz"))
    sim = MskArmSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = ArmRenderer(sim.scene)
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    for k in range(steps):
        depth = sim.depth_camera()
        x = depth[None, None, :, :].astype(np.float32)
        act = model.predict(x)[0]
        act = np.clip(act, 0.0, 1.0)
        sim.step(act)
        pose = sim.pose()
        trail.append((float(pose[0]), float(pose[1])))
        if k % capture_every == 0:
            frames.append(renderer.render(
                sim.q, activations=sim.muscle_activations(), goal=sim.goal(),
                trail=trail, hud=_hud(sim), mode="END-TO-END CNN",
                key_label="NN", message="前视深度图 -> 卷积网络 -> 4 肌肉激活",
                time=sim.time, torque=sim.max_torque, reached=sim.reached,
                collisions=sim.collisions,
                ee_error=math.hypot(pose[0] - sim.goal()[0],
                                    pose[1] - sim.goal()[1])))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps)
    frames[len(frames) // 2].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "e2e", "scene": scene, "frames": len(frames),
            "bytes": size, "collisions": sim.collisions, "path": out}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--mode", default="teleop", choices=["teleop", "reach", "e2e"])
    p.add_argument("--scene", default="reach")
    p.add_argument("--seconds", type=float, default=6.6)
    p.add_argument("--fps", type=int, default=8)
    p.add_argument("--out", default="")
    args = p.parse_args()
    out = args.out or os.path.join(C.ASSETS_DIR, "%s.gif" % args.mode)
    if args.mode == "teleop":
        meta = record_teleop(args.scene, args.seconds, out, args.fps)
    elif args.mode == "reach":
        meta = record_reach(args.scene, args.seconds, out, args.fps)
    else:
        meta = record_e2e(args.scene, args.seconds, out, args.fps)
    os.makedirs(C.RESULTS_DIR, exist_ok=True)
    with open(os.path.join(C.RESULTS_DIR, "gif_%s.json" % args.mode), "w") as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False)
    print(json.dumps(meta, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
