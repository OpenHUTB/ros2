"""生成文档用图：场景快照与训练损失曲线。"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_core import USVSim, ANGLE_MIN, ANGLE_MAX, MAX_RANGE, thrust_from_twist  # noqa: E402
from occupancy import OccupancyMap                                                # noqa: E402
from main import draw                                                             # noqa: E402


def scene_snapshot(path):
    sim = USVSim(seed=7)
    obs = sim.reset(start=(-18.0, -16.0, 0.0), goal=(18.0, 18.0), randomize=False)
    occ = OccupancyMap()
    trail = []
    for k in range(260):
        pose = sim.pose()
        occ.update(pose, obs["scan"], ANGLE_MIN, ANGLE_MAX, MAX_RANGE)
        # 朝目标直线推进，仅用于把地图建起来
        gb, gd = sim.goal_in_body()
        alpha = float(np.arctan2(gb[1], gb[0]))
        v = 1.0
        w = float(np.clip(0.9 * alpha, -1.0, 1.0))
        tl, tr = thrust_from_twist(v, w)
        obs = sim.step(tl, tr)
        trail.append(sim.pose()[:2])
        if sim.collided or obs["goal_dist"] < 1.5:
            break

    fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=120)
    draw(ax, sim, occ, trail, "MuJoCo USV  ·  online occupancy grid")
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print("scene ->", path)


def loss_curve(loss_path, out_path):
    h = np.load(loss_path)
    fig, ax = plt.subplots(figsize=(6.4, 3.6), dpi=130)
    ax.plot(np.arange(1, len(h) + 1), h, "o-", ms=3, color="#1f77b4")
    ax.set_xlabel("epoch")
    ax.set_ylabel("training MSE")
    ax.set_title("MLP training loss")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print("loss ->", out_path)


if __name__ == "__main__":
    pkg = os.path.join(HERE, "..")
    outdir = os.path.join(pkg, "results")
    os.makedirs(outdir, exist_ok=True)
    scene_snapshot(os.path.join(outdir, "scene.png"))
    loss_curve(os.path.join(pkg, "models", "train_loss.npy"),
               os.path.join(outdir, "loss.png"))
