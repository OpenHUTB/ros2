"""移动障碍避让可视化与 GIF 导出。"""

import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_core import (DynSim, HALF_X, HALF_Y, GOAL, GOAL_R, MAX_RANGE,
                      ANGLE_MIN, ANGLE_MAX, thrust_from_twist)
from oracle import build_path, DynExpert
from nn import MLP


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "dyn_planner.npz"))
        z = np.load(os.path.join(model_dir, "dyn_norm.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.v = self.w = None

    def act(self, sim, obs, prev):
        feat = np.concatenate([obs["scan"], prev,
                               [sim.goal_bearing(), min(sim.goal_distance(), 20.0), obs["speed"]]])
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
    ax.set_facecolor("#f4f3ef")

    ax.add_patch(Rectangle((-HALF_X - 0.3, -HALF_Y - 0.3), 2 * HALF_X + 0.6, 2 * HALF_Y + 0.6,
                           fc="none", ec="#3a3a3a", lw=2.2, zorder=5))
    for o in sim.obstacles:
        ax.add_patch(Rectangle((o["x"] - o["hx"], o["y"] - o["hy"]), 2 * o["hx"], 2 * o["hy"],
                               fc="#8d7b63", ec="#5c5145", zorder=4))
    for mx, my, _, _ in sim.movers():
        ax.add_patch(Circle((mx, my), sim.mover_radius(), fc="#d33f2f", ec="#8c1f14", zorder=5))
    ax.add_patch(Circle(GOAL, GOAL_R, fc="#3fbf7f", alpha=0.45, zorder=3))

    if path is not None:
        ax.plot(path[:, 0], path[:, 1], "--", color="#b8b8b8", lw=0.9, zorder=2)

    x, y, yaw = sim.pose()
    hull = np.array([[0.35, 0.30], [0.35, -0.30], [-0.35, -0.30], [-0.35, 0.30]])
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    ax.add_patch(Polygon(hull @ R.T + np.array([x, y]), closed=True,
                         fc="#1f6fb2", ec="#0d3d66", lw=1.0, zorder=7))

    scan = sim.lidar()
    ang = yaw + np.linspace(ANGLE_MIN, ANGLE_MAX, scan.shape[0])
    for a, r in zip(ang[::3], scan[::3]):
        ax.plot([x, x + r * np.cos(a)], [y, y + r * np.sin(a)],
                color="#2a9fd6", lw=0.3, alpha=0.30, zorder=3)

    if trail:
        t = np.array(trail)
        ax.plot(t[:, 0], t[:, 1], color="#c0392b", lw=1.3, zorder=6)
    ax.set_title(title, fontsize=9)
    ax.set_xticks([])
    ax.set_yticks([])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["nn", "expert"], default="nn")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--steps", type=int, default=1500)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--save-gif", type=str, default="")
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--dpi", type=int, default=75)
    args = ap.parse_args()

    model_dir = os.path.join(HERE, "..", args.model_dir)
    rng = np.random.default_rng(args.seed)
    sim = DynSim(seed=args.seed)
    obs = sim.reset(None, rng)
    path = build_path(sim.obstacles, sim.pose()[:2], GOAL)
    expert = DynExpert(path) if args.mode == "expert" else None
    pilot = NNPilot(model_dir) if args.mode == "nn" else None
    prev = obs["scan"].copy()

    fig, ax = plt.subplots(figsize=(5.2, 4.2), dpi=args.dpi)
    frames, trail = [], []
    res = "timeout"
    for k in range(args.steps):
        if sim.collided:
            res = "collision@%d" % k
            break
        if sim.reached():
            res = "reached@%d" % k
            break
        if args.mode == "nn":
            v, w = pilot.act(sim, obs, prev)
        else:
            v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["movers"], obs["scan"])
        prev = obs["scan"].copy()
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        trail.append(sim.pose()[:2])
        if args.save_gif and k % args.every == 0:
            draw(ax, sim, trail, path, "%s  step=%d  d=%.1fm" % (args.mode, k, sim.goal_distance()))
            fig.canvas.draw()
            frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())

    print("result=%s steps=%d goal_dist=%.2f" % (res, k + 1, sim.goal_distance()))
    if args.save_gif and frames:
        from PIL import Image
        imgs = [Image.fromarray(f) for f in frames]
        imgs[0].save(args.save_gif, save_all=True, append_images=imgs[1:],
                     duration=70, loop=0, optimize=True)
        print("gif:", args.save_gif, os.path.getsize(args.save_gif) // 1024, "KB")


if __name__ == "__main__":
    main()
