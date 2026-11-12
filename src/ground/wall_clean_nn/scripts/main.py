#!/usr/bin/env python3
"""沿墙清扫可视化与 GIF 导出。"""

import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

STOP_COV = 0.95      # 覆盖率达标即停（与 evaluate.py 的 COV_TARGET 一致）

HERE = os.path.dirname(os.path.abspath(__file__))


def _pkg_paths():
    """返回 (scripts_dir, models_dir)。

    源码空间用本文件目录；安装空间（rosrun 调用 lib/<包>/ 下的副本）经 rospkg
    定位 share/<包>，否则 from sim_core import 会失败、模型目录也会指到 lib/ 下。
    """
    scripts = HERE
    models = os.path.abspath(os.path.join(HERE, "..", "models"))
    try:
        import rospkg
        pkg = rospkg.RosPack().get_path("wall_clean_nn")
        cand = os.path.join(pkg, "scripts")
        if os.path.isdir(cand):
            scripts = cand
            models = os.path.join(pkg, "models")
    except Exception:
        pass
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    return scripts, models


SCRIPTS_DIR, MODELS_DIR = _pkg_paths()

from sim_core import (CoverSim, HALF_X, HALF_Y, RES, NX, NY, BRUSH_R,
                      ANGLE_MIN, ANGLE_MAX, thrust_from_twist)
from oracle import build_wall_path, WallExpert, sample_start
from nn import MLP


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "wall_planner.npz"))
        z = np.load(os.path.join(model_dir, "wall_norm.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.v = self.w = None

    def act(self, sim, obs):
        cov = sim.local_coverage(size=11).reshape(-1)
        feat = np.concatenate([obs["scan"], cov, [obs["speed"]]])
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        v = float(np.clip(yn[0] * self.ys[0] + self.ym[0], -0.3, 1.0))
        w = float(np.clip(yn[1] * self.ys[1] + self.ym[1], -1.2, 1.2))
        a = 0.5
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)
        return self.v, self.w


def draw(ax, sim, trail, path, title):
    ax.clear()
    ax.set_xlim(-HALF_X - 0.4, HALF_X + 0.4)
    ax.set_ylim(-HALF_Y - 0.4, HALF_Y + 0.4)
    ax.set_aspect("equal")
    ax.set_facecolor("#f2f0ea")

    # 已清扫栅格
    for i in range(NY):
        for j in range(NX):
            if sim.visited[i, j]:
                c = sim.cell_center(i, j)
                ax.add_patch(Rectangle((c[0] - RES / 2, c[1] - RES / 2), RES, RES,
                                       fc="#7fd1ae", ec="none", alpha=0.55, zorder=1))
    for o in sim.obstacles:
        ax.add_patch(Rectangle((o["x"] - o["hx"], o["y"] - o["hy"]), 2 * o["hx"], 2 * o["hy"],
                               fc="#8d7b63", ec="#5c5145", zorder=4))
    ax.add_patch(Rectangle((-HALF_X - 0.3, -HALF_Y - 0.3), 2 * HALF_X + 0.6, 2 * HALF_Y + 0.6,
                           fc="none", ec="#3a3a3a", lw=2.2, zorder=5))

    if path is not None:
        ax.plot(path[:, 0], path[:, 1], "--", color="#b0b0b0", lw=0.8, zorder=2)

    x, y, yaw = sim.pose()
    hull = np.array([[0.35, 0.30], [0.35, -0.30], [-0.35, -0.30], [-0.35, 0.30]])
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    ax.add_patch(Polygon(hull @ R.T + np.array([x, y]), closed=True,
                         fc="#1f6fb2", ec="#0d3d66", lw=1.0, zorder=7))
    bx, by = x + 0.42 * c, y + 0.42 * s
    ax.add_patch(plt.Circle((bx, by), BRUSH_R, fc="#e8a33d", alpha=0.45, zorder=6))

    scan = sim.lidar()
    ang = yaw + np.linspace(ANGLE_MIN, ANGLE_MAX, scan.shape[0])
    for a, r in zip(ang[::2], scan[::2]):
        ax.plot([x, x + r * np.cos(a)], [y, y + r * np.sin(a)],
                color="#2a9fd6", lw=0.3, alpha=0.35, zorder=3)

    if trail:
        t = np.array(trail)
        ax.plot(t[:, 0], t[:, 1], color="#c0392b", lw=1.2, zorder=6)
    ax.set_title(title, fontsize=9)
    ax.set_xticks([])
    ax.set_yticks([])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["nn", "expert"], default="nn")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--steps", type=int, default=12000)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--save-gif", type=str, default="")
    ap.add_argument("--every", type=int, default=40)
    ap.add_argument("--dpi", type=int, default=70)
    args = ap.parse_args()

    model_dir = MODELS_DIR if args.model_dir == "models" else args.model_dir
    sim = CoverSim(seed=args.seed)
    rng = np.random.default_rng(args.seed)
    start = sample_start(rng, sim.obstacles)
    obs = sim.reset(start)
    path, _ = build_wall_path(sim.obstacles, start[:2])
    expert = WallExpert(path) if args.mode == "expert" else None
    pilot = NNPilot(model_dir) if args.mode == "nn" else None

    fig, ax = plt.subplots(figsize=(4.4, 3.9), dpi=args.dpi)
    frames, trail = [], []
    res = "timeout"
    for k in range(args.steps):
        if sim.collided:
            res = "collision@%d" % k
            break
        if sim.wall_coverage() >= STOP_COV:
            res = "done@%d" % k
            break
        if args.mode == "nn":
            v, w = pilot.act(sim, obs)
        else:
            v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["scan"])
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        trail.append(sim.pose()[:2])
        if args.save_gif and k % args.every == 0:
            draw(ax, sim, trail, path, "%s  step=%d  wall cov=%.0f%%" % (args.mode, k, 100 * sim.wall_coverage()))
            fig.canvas.draw()
            frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())

    print("result=%s steps=%d wall_cov=%.1f%% err=%.3f" %
          (res, k + 1, 100 * sim.wall_coverage(), abs(sim.wall_error() - 0.85)))
    if args.save_gif and frames:
        from PIL import Image
        imgs = [Image.fromarray(f) for f in frames]
        imgs[0].save(args.save_gif, save_all=True, append_images=imgs[1:],
                     duration=80, loop=0, optimize=True)
        print("gif:", args.save_gif, os.path.getsize(args.save_gif) // 1024, "KB")


if __name__ == "__main__":
    main()
