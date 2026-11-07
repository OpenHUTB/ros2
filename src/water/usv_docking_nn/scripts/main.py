"""靠泊可视化与 GIF 导出。"""

import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

from sim_core import DockSim, ANGLE_MIN, ANGLE_MAX, thrust_from_twist, BERTH_X, TOL_Y
from oracle import DockExpert, build_path
from nn import MLP


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "dock_planner.npz"))
        z = np.load(os.path.join(model_dir, "dock_norm.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.v = self.w = None

    def act(self, obs):
        e = obs["berth"]
        feat = np.concatenate([obs["scan"],
                               [np.clip(e[0], -3, 3), np.clip(e[1], -1, 1),
                                np.clip(e[2], -20, 20), obs["speed"]]])
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        v = float(np.clip(yn[0] * self.ys[0] + self.ym[0], 0.0, 1.5))
        w = float(np.clip(yn[1] * self.ys[1] + self.ym[1], -0.8, 0.8))
        a = 0.5
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)
        return self.v, self.w


def draw(ax, sim, trail, title):
    ax.clear()
    ax.set_xlim(-10, 22)
    ax.set_ylim(-9, 9)
    ax.set_aspect("equal")
    ax.set_facecolor("#0d2433")

    ax.add_patch(Rectangle((19.0, -10), 3.0, 20, fc="#6b6558"))
    ax.add_patch(Rectangle((17.9, 3.55), 1.2, 0.7, fc="#6b6558"))
    ax.add_patch(Rectangle((17.9, -4.25), 1.2, 0.7, fc="#6b6558"))
    ax.add_patch(Rectangle((18.65, -3.55), 0.35, 7.1, fc="#d4b83a"))
    ax.axhline(0, color="#3fd0a0", ls="--", lw=0.8, alpha=0.6)
    ax.plot(BERTH_X, 0, marker="x", ms=10, color="#ff6b6b")

    x, y, yaw = sim.pose()
    hull = np.array([[0.75, 0.35], [0.75, -0.35], [-0.75, -0.35], [-0.75, 0.35]])
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    ax.add_patch(Polygon(hull @ R.T + np.array([x, y]), closed=True,
                         fc="#e05a12", ec="#ffd9b0", lw=1.2, zorder=6))

    scan = sim.lidar()
    ang = yaw + np.linspace(ANGLE_MIN, ANGLE_MAX, scan.shape[0])
    for a, r in zip(ang, scan):
        ax.plot([x, x + r * np.cos(a)], [y, y + r * np.sin(a)],
                color="#39d0ff", lw=0.3, alpha=0.30, zorder=2)

    if trail:
        t = np.array(trail)
        ax.plot(t[:, 0], t[:, 1], color="#ffd166", lw=1.6, zorder=5)
    ax.set_title(title, fontsize=9)
    ax.set_xticks([])
    ax.set_yticks([])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["nn", "expert"], default="nn")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--steps", type=int, default=1100)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--save-gif", type=str, default="")
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--dpi", type=int, default=85)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.join(here, "..", args.model_dir)

    sim = DockSim(seed=args.seed)
    obs = sim.reset(seed=args.seed)
    expert = DockExpert(build_path(sim.start)) if args.mode == "expert" else None
    pilot = NNPilot(model_dir) if args.mode == "nn" else None

    fig, ax = plt.subplots(figsize=(5.4, 4.2), dpi=args.dpi)
    frames, trail = [], []
    res = "timeout"
    for k in range(args.steps):
        if sim.collided:
            res = "collision@%d" % k
            break
        if sim.docked():
            res = "docked@%d" % k
            break
        if args.mode == "nn":
            v, w = pilot.act(obs)
        else:
            v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate())
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        trail.append(sim.pose()[:2])
        if args.save_gif and k % args.every == 0:
            draw(ax, sim, trail, "%s  step=%d  d=%.1fm" % (args.mode, k, obs["berth"][2]))
            fig.canvas.draw()
            frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())

    print("result=%s steps=%d final=(%.2f, %.2f, %.2f)"
          % (res, k + 1, *sim.pose()))
    if args.save_gif and frames:
        from PIL import Image
        imgs = [Image.fromarray(f) for f in frames]
        imgs[0].save(args.save_gif, save_all=True, append_images=imgs[1:],
                     duration=70, loop=0, optimize=True)
        print("gif:", args.save_gif, os.path.getsize(args.save_gif) // 1024, "KB")


if __name__ == "__main__":
    main()
