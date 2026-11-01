"""录制演示 GIF（离屏渲染，无需显示器/GPU）。

用法::

    python tools/record_gif.py --mode teleop --scene warehouse --seconds 8
    python tools/record_gif.py --mode nav   --scene obstacle_field --seconds 25
    python tools/record_gif.py --mode e2e   --scene corridor --seconds 20
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from humanoid_mujoco_slam_nn_nav.gait import GaitParams, Walker  # noqa: E402
from humanoid_mujoco_slam_nn_nav.nav import NavConfig, Navigation  # noqa: E402
from humanoid_mujoco_slam_nn_nav.sim import HumanoidSim  # noqa: E402

W, H = 480, 360


def render_frame(sim, cam="track"):
    return sim.follow_view(W, H, distance=2.6, azimuth=125, elevation=-12)


def make_gif(frames, path, fps=10, max_width=W):
    imgs = [Image.fromarray(f) for f in frames[::max(1, round(30 / fps))]]
    if not imgs:
        return
    imgs[0].save(path, save_all=True, append_images=imgs[1:], optimize=True,
                 duration=int(1000 / fps), loop=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="teleop", choices=["teleop", "nav", "e2e"])
    ap.add_argument("--scene", default="warehouse")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--out", default="")
    ap.add_argument("--fps", type=int, default=10)
    args = ap.parse_args()

    os.makedirs("docs/assets", exist_ok=True)
    out = args.out or f"docs/assets/{args.mode}.gif"

    sim = HumanoidSim(scene=args.scene, render=True)
    frames = []
    dt = 1 / 30.0

    if args.mode == "teleop":
        gp = GaitParams(pelvis_mode="feet", pelvis_height=0.70, duty=0.78, period=1.15,
                        side_shift=0.02, foot_spread=0.07, step_height=0.08,
                        capture_gain=0.35, capture_max=0.08, com_kp=1.0, com_kd=0.1,
                        com_max=0.04, roll_gain=0.0, pitch_gain=0.0, ankle_kp=0.0, ankle_kd=0.0)
        sx, sy, syaw = sim.layout.start
        walker = Walker(gp, sx, sy, syaw)
        sim.reset()
        t = 0.0
        while t < args.seconds:
            st = sim.state()
            v = 0.20 if 1.0 < t < args.seconds - 1.5 else 0.0
            w = 0.35 if args.seconds * 0.55 < t < args.seconds * 0.75 else 0.0
            walker.command(v, w)
            feet = {"l": tuple(sim.ankle_pos("l")[:2]), "r": tuple(sim.ankle_pos("r")[:2])}
            q = walker.update(dt, actual_pos=st["pos"], actual_yaw=st["yaw"],
                              roll=st["roll"], pitch=st["pitch"], vel=st["vel"], feet_xy=feet)
            sim.set_targets(q)
            sim.step(33)
            t += dt
            frames.append(render_frame(sim))
    else:
        cfg = NavConfig(settle_period=9.0, settle_time=1.4, v_max=0.25, inflation=0.30)
        nav = Navigation(sim, cfg, policy="e2e" if args.mode == "e2e" else "classical",
                         with_depth=(args.mode == "e2e"))
        t = 0.0
        while t < args.seconds and nav.mode != "done":
            for _ in range(3):
                nav.step()
            t = nav.t
            frames.append(render_frame(sim))
            if sim.is_fallen():
                break

    make_gif(frames, out, fps=args.fps)
    size_mb = os.path.getsize(out) / 1e6
    print(f"saved {out} frames={len(frames)} size={size_mb:.2f} MB")


if __name__ == "__main__":
    main()
