"""USV 交互式仿真：专家策略 / 神经网络策略 / 键盘手动控制。

用法示例：
    python3 main.py --mode nn                 # 神经网络自主航行（弹窗）
    python3 main.py --mode manual             # 键盘控制（W/S 油门，A/D 转向）
    python3 main.py --mode nn --save-gif demo.gif --duration 300
"""

import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon

from sim_core import USVSim, ANGLE_MIN, ANGLE_MAX, MAX_RANGE, thrust_from_twist
from occupancy import OccupancyMap
from oracle import GridPlanner
from nn import MLP
from policy import NNPolicy

CROP = 21


def _read_key():
    """非阻塞读取一个字符（Linux 终端）。"""
    import termios
    import tty
    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        return ""
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if r:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ""


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "planner_mlp.npz"))
        z = np.load(os.path.join(model_dir, "norm_stats.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]

    def act(self, feat):
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        return (float(np.clip(yn[0] * self.ys[0] + self.ym[0], 0.0, 2.0)),
                float(np.clip(yn[1] * self.ys[1] + self.ym[1], -1.2, 1.2)))


def draw(ax, sim, occ, trail, title):
    ax.clear()
    ax.set_xlim(-25, 25)
    ax.set_ylim(-25, 25)
    ax.set_aspect("equal")
    ax.set_facecolor("#0e2a3a")

    prob = occ.probability()
    known = np.abs(occ.logodds) > 0.05
    pm = np.ma.masked_where(~known, prob)
    ax.imshow(pm, extent=[-25, 25, -25, 25], origin="lower",
              cmap="inferno", vmin=0.0, vmax=1.0, alpha=0.65)

    for o in sim.obstacles:
        if o["type"] == "circle":
            ax.add_patch(Circle((o["x"], o["y"]), o["r"], color="#7a6a4f", zorder=3))
        else:
            ax.add_patch(Rectangle((o["x"] - o["hx"], o["y"] - o["hy"]),
                                   2 * o["hx"], 2 * o["hy"], color="#7a6a4f", zorder=3))

    ax.add_patch(Rectangle((-25, -25), 50, 50, fill=False, ec="#9fb6c4", lw=2, zorder=4))

    x, y, yaw = sim.pose()
    hull = np.array([[0.78, 0.36], [0.78, -0.36], [-0.75, -0.36], [-0.75, 0.36]])
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    ax.add_patch(Polygon(hull @ R.T + np.array([x, y]), closed=True,
                         fc="#e05a12", ec="#ffd9b0", lw=1.2, zorder=6))

    # 激光雷达射线
    scan = sim.lidar()
    angles = yaw + np.linspace(ANGLE_MIN, ANGLE_MAX, scan.shape[0])
    for a, r in zip(angles, scan):
        ax.plot([x, x + r * np.cos(a)], [y, y + r * np.sin(a)],
                color="#39d0ff", lw=0.35, alpha=0.35, zorder=2)

    if trail:
        t = np.array(trail)
        ax.plot(t[:, 0], t[:, 1], color="#ffd166", lw=1.6, zorder=5)

    ax.plot(sim.goal[0], sim.goal[1], marker="*", ms=18, color="#4dff88", zorder=7)
    ax.set_title(title, fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["nn", "expert", "manual"], default="nn")
    ap.add_argument("--duration", type=int, default=600)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--model-dir", type=str, default="models")
    ap.add_argument("--save-gif", type=str, default="")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--every", type=int, default=2)
    ap.add_argument("--dpi", type=int, default=110)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.join(here, "..", args.model_dir)

    sim = USVSim(seed=args.seed)
    planner = GridPlanner(sim.obstacles)
    occ = OccupancyMap()
    obs = sim.reset(randomize=(args.mode != "manual"))
    if args.mode == "manual":
        obs = sim.reset(start=(-18.0, -18.0, 0.0), goal=(18.0, 18.0), randomize=False)

    pilot = NNPolicy(model_dir) if args.mode == "nn" else None
    path = planner.plan(sim.start[:2], sim.goal) if args.mode == "expert" else None

    fig, ax = plt.subplots(figsize=(5.0, 5.0), dpi=args.dpi)
    frames = []
    trail = []
    v_cmd, w_cmd = 0.0, 0.0
    result = "timeout"

    for step in range(args.duration):
        pose = sim.pose()
        occ.update(pose, obs["scan"], ANGLE_MIN, ANGLE_MAX, MAX_RANGE)

        if args.mode == "manual":
            k = _read_key().lower()
            if k == "w":
                v_cmd = min(v_cmd + 0.15, 1.8)
            elif k == "s":
                v_cmd = max(v_cmd - 0.15, 0.0)
            elif k == "a":
                w_cmd = min(w_cmd + 0.12, 1.2)
            elif k == "d":
                w_cmd = max(w_cmd - 0.12, -1.2)
            elif k == "q":
                break
            else:
                v_cmd *= 0.995
                w_cmd *= 0.97
        elif args.mode == "expert":
            v_cmd, w_cmd = planner.pure_pursuit(pose, path)
        else:
            feat = np.concatenate([occ.crop(pose, CROP).reshape(-1),
                                   [obs["goal_body"][0], obs["goal_body"][1], obs["goal_dist"]]])
            v_cmd, w_cmd = pilot.act(feat, obs["scan"])

        tl, tr = thrust_from_twist(v_cmd, w_cmd)
        obs = sim.step(tl, tr)
        trail.append(sim.pose()[:2])

        if args.save_gif and step % args.every == 0:
            draw(ax, sim, occ, trail, "USV  mode=%s  step=%d" % (args.mode, step))
            fig.canvas.draw()
            frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())

        if sim.collided:
            result = "collision at step %d" % step
            break
        if obs["goal_dist"] < 1.5:
            result = "reached goal at step %d" % step
            break

    print("result: %s | steps=%d | coverage=%.1f%%" % (result, step + 1, 100 * occ.covered_ratio()))

    if args.save_gif and frames:
        from PIL import Image
        imgs = [Image.fromarray(f) for f in frames]
        out = args.save_gif
        imgs[0].save(out, save_all=True, append_images=imgs[1:], duration=80, loop=0, optimize=True)
        print("gif saved:", out, os.path.getsize(out) // 1024, "KB")


if __name__ == "__main__":
    main()
