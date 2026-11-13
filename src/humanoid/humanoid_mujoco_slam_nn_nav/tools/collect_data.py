"""采集行为克隆数据集：用经典导航栈（A* + 纯跟踪）当"老师"。

输出 ``data/dataset.npz``：

* ``scan``       (N,180) 带噪激光（用于感知网络输入）
* ``scan_clean`` (N,180) 无噪激光（感知网络标签）
* ``patch``      (N,11,11) 以机器人为中心的局部占据栅格（下采样）
* ``goal``       (N,3) 目标在体坐标系下的 (dx, dy, 距离)
* ``cmd``        (N,2) 老师给出的 (v, ω)
* ``depth``      (N,24,32) 深度相机图像（米）
* ``dir``        (N,2) 老师路径方向 (cosφ, sinφ)
* ``pose``       (N,3) 真值位姿（仅用于统计，不进网络）
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from humanoid_mujoco_slam_nn_nav.mapping import OccupancyGrid, local_patch  # noqa: E402
from humanoid_mujoco_slam_nn_nav.nav import NavConfig, Navigation  # noqa: E402
from humanoid_mujoco_slam_nn_nav.sim import HumanoidSim  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--scene", default="warehouse")
    ap.add_argument("--seconds", type=float, default=55.0)
    ap.add_argument("--out", default="data/dataset.npz")
    args = ap.parse_args()

    cfg = NavConfig(settle_period=9.0, settle_time=1.4, v_max=0.25, inflation=0.30)
    rng = np.random.default_rng(7)
    buf = {k: [] for k in ("scan", "scan_clean", "patch", "goal", "cmd", "depth", "dir", "pose")}

    scenes = [args.scene, "obstacle_field", "corridor"][:args.episodes]
    for scene in scenes:
        sim = HumanoidSim(scene=scene)
        nav = Navigation(sim, cfg, with_depth=True)
        dx, dy, dyaw = sim.layout.start
        t_next = 0.0
        print(f"[collect] {scene}")
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

            gx = nav.goal[0] - pose[0]
            gy = nav.goal[1] - pose[1]
            c, s = np.cos(pose[2]), np.sin(pose[2])
            local = np.array([c * gx + s * gy, -s * gx + c * gy])
            goal = np.array([local[0], local[1], float(np.hypot(gx, gy))])

            if nav.path:
                li = min(nav.path_idx + 3, len(nav.path) - 1)
                px, py = nav.path[li]
                hx, hy = px - pose[0], py - pose[1]
                ang = np.arctan2(-s * hx + c * hy, c * hx + s * hy)
                direction = np.array([np.cos(ang), np.sin(ang)])
            else:
                direction = np.array([1.0, 0.0])

            depth = nav.depth.depth(sim.data) if nav.depth is not None else np.zeros((24, 32))
            buf["scan"].append(noisy.astype(np.float32))
            buf["scan_clean"].append(clean.astype(np.float32))
            buf["patch"].append(patch.astype(np.float32))
            buf["goal"].append(goal.astype(np.float32))
            buf["cmd"].append(np.array([nav.v, nav.w], dtype=np.float32))
            buf["depth"].append(depth.astype(np.float32))
            buf["dir"].append(direction.astype(np.float32))
            buf["pose"].append(np.array(pose, dtype=np.float32))
        print(f"  frames={len(buf['scan'])} fallen={sim.is_fallen()}")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    arrays = {k: np.stack(v) for k, v in buf.items()}
    np.savez_compressed(args.out, **arrays)
    print(f"saved {args.out} frames={len(arrays['scan'])}")


if __name__ == "__main__":
    main()
