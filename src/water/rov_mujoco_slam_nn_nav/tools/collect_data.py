"""采集无人机行为克隆数据（经典栈当老师）。"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rov_mujoco_slam_nn_nav.mapping import local_patch  # noqa: E402
from rov_mujoco_slam_nn_nav.nav import NavConfig, Navigation  # noqa: E402
from rov_mujoco_slam_nn_nav.sim import ROVSim  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--out", default="data/dataset.npz")
    args = ap.parse_args()

    cfg = NavConfig()
    rng = np.random.default_rng(3)
    buf = {k: [] for k in ("scan", "scan_clean", "patch", "goal", "cmd", "depth", "dir", "pose", "scene")}

    reps = [("pool", None), ("pipe_field", None), ("trench", None),
            ("pool", (2.6, 1.4)), ("pipe_field", (2.5, -1.2)), ("trench", (2.4, 1.0))]
    for rep_i, (scene, alt_goal) in enumerate(reps):
        sim = ROVSim(scene=scene)
        nav = Navigation(sim, cfg, with_depth=True)
        if alt_goal is not None:
            nav.goal = np.array(alt_goal, dtype=float)
            sim.goal = nav.goal
        t_next = 0.0
        print(f"[collect] {scene} goal={tuple(nav.goal)}")
        while nav.t < args.seconds and nav.mode != "done" and not sim.is_fallen():
            nav.step()
            if nav.t < t_next:
                continue
            t_next = nav.t + 0.1
            st = sim.state()
            clean = nav.lidar.scan(sim.data)
            noisy = np.clip(clean + rng.normal(0, 0.03, clean.shape), 0.05, 8.0)
            pose = st["pos"][0], st["pos"][1], st["yaw"]
            patch = local_patch(nav.grid, pose)

            gx, gy = nav.goal[0] - pose[0], nav.goal[1] - pose[1]
            c, s = math.cos(pose[2]), math.sin(pose[2])
            goal = np.array([c * gx + s * gy, -s * gx + c * gy, float(np.hypot(gx, gy))])

            if nav.path:
                li = min(nav.path_idx + 3, len(nav.path) - 1)
                hx, hy = nav.path[li][0] - pose[0], nav.path[li][1] - pose[1]
                ang = math.atan2(-s * hx + c * hy, c * hx + s * hy)
                direction = np.array([math.cos(ang), math.sin(ang)])
            else:
                direction = np.array([1.0, 0.0])

            v = math.hypot(nav.vx, nav.vy)
            depth = nav.depth.depth(sim.data)
            buf["scan"].append(noisy.astype(np.float32))
            buf["scan_clean"].append(clean.astype(np.float32))
            buf["patch"].append(patch.astype(np.float32))
            buf["goal"].append(goal.astype(np.float32))
            buf["cmd"].append(np.array([nav.vx, nav.vy, nav.w], dtype=np.float32))
            buf["depth"].append(depth.astype(np.float32))
            buf["dir"].append(direction.astype(np.float32))
            buf["pose"].append(np.array(pose, dtype=np.float32))
            buf["scene"].append(np.float32(rep_i))
        print(f"  frames={len(buf['scan'])} fallen={sim.is_fallen()}")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    np.savez_compressed(args.out, **{k: np.stack(v) for k, v in buf.items()})
    print(f"saved {args.out} frames={len(buf['scan'])}")


if __name__ == "__main__":
    main()


