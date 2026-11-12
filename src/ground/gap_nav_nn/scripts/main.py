"""狭窄通道通行可视化与 GIF 导出。"""

import argparse, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_core import (GapSim, HALF_X, HALF_Y, GOAL_R, MAX_RANGE,
                      ANGLE_MIN, ANGLE_MAX, thrust_from_twist, GAPS)
from oracle import build_path, GapExpert, choose_gap
from pilot import NNPilot, features



def draw(ax, sim, trail, gap, title):
    ax.clear()
    ax.set_xlim(-HALF_X - 0.4, HALF_X + 0.4)
    ax.set_ylim(-HALF_Y - 0.4, HALF_Y + 0.4)
    ax.set_aspect("equal")
    ax.set_facecolor("#f4f3ef")

    ax.add_patch(Rectangle((-HALF_X - 0.3, -HALF_Y - 0.3), 2 * HALF_X + 0.6, 2 * HALF_Y + 0.6,
                           fc="none", ec="#3a3a3a", lw=2.2, zorder=5))
    for w in sim.walls:
        ax.add_patch(Rectangle((w["x"] - w["hx"], w["y"] - w["hy"]), 2 * w["hx"], 2 * w["hy"],
                               fc="#b09a74", ec="#6b5a3e", zorder=4))
    for g in GAPS:
        ax.plot([0, 0], [g["lo"], g["hi"]], color="#2e8b57" if (g["hi"] - g["lo"]) > 0.9 else "#c0392b",
                lw=3.0, alpha=0.5, zorder=3)
    ax.add_patch(Circle(sim.goal, GOAL_R, fc="#3fbf7f", alpha=0.45, zorder=3))

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
    ap.add_argument("--steps", type=int, default=1200)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--save-gif", type=str, default="")
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--dpi", type=int, default=75)
    args = ap.parse_args()

    model_dir = os.path.join(HERE, "..", args.model_dir)
    rng = np.random.default_rng(args.seed)
    sim = GapSim(seed=args.seed)
    obs = sim.reset(None, None, rng)
    path, gap = build_path(sim.start, sim.goal)
    expert = GapExpert(path) if args.mode == "expert" else None
    pilot = NNPilot(model_dir) if args.mode == "nn" else None

    fig, ax = plt.subplots(figsize=(5.2, 4.4), dpi=args.dpi)
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
            v, w = pilot.act(features(sim, obs))
        else:
            v, w = expert.act(sim.pose(), sim.speed(), sim.yaw_rate(), obs["scan"])
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        trail.append(sim.pose()[:2])
        if args.save_gif and k % args.every == 0:
            draw(ax, sim, trail, gap, "%s  step=%d  d=%.1fm" % (args.mode, k, sim.goal_distance()))
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
