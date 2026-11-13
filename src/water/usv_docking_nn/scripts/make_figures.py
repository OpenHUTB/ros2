"""生成文档配图：训练损失曲线 + 专家/神经网络轨迹对比。"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_core import DockSim, thrust_from_twist, BERTH_X                      # noqa: E402
from oracle import DockExpert, build_path                                     # noqa: E402
from evaluate import NNPilot                                                  # noqa: E402


def loss_curve(path, out):
    h = np.load(path)
    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=130)
    ax.plot(np.arange(1, len(h) + 1), h, "o-", ms=3, color="#1f77b4")
    ax.set_xlabel("epoch")
    ax.set_ylabel("training MSE")
    ax.set_title("MLP training loss (docking)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print("loss ->", out)


def trajectories(model_dir, out, seeds=(0, 1, 2)):
    fig, ax = plt.subplots(figsize=(6.8, 4.6), dpi=130)
    ax.add_patch(Rectangle((19.0, -10), 3.0, 20, fc="#9a9384"))
    ax.add_patch(Rectangle((17.9, 3.55), 1.2, 0.7, fc="#9a9384"))
    ax.add_patch(Rectangle((17.9, -4.25), 1.2, 0.7, fc="#9a9384"))
    ax.axhline(0, color="k", ls="--", lw=0.8, alpha=0.4)

    colors = ["#1f77b4", "#d62728", "#2ca02c"]
    for i, s in enumerate(seeds):
        for name, mk in (("expert", None), ("nn", "nn")):
            sim = DockSim(seed=s)
            sim.reset(seed=s)
            pol = NNPilot(model_dir) if mk == "nn" else None
            ex = DockExpert(build_path(sim.start)) if mk is None else None
            xs = []
            for k in range(1100):
                if sim.collided or sim.docked():
                    break
                if pol is not None:
                    v, w = pol.act(sim.observation())
                else:
                    v, w = ex.act(sim.pose(), sim.speed(), sim.yaw_rate())
                tl, tr = thrust_from_twist(v, w)
                sim.step(tl, tr)
                xs.append(sim.pose()[:2])
            if xs:
                a = np.array(xs)
                ax.plot(a[:, 0], a[:, 1], color=colors[i],
                        ls="-" if mk == "nn" else "--", lw=1.3,
                        label="seed %d %s" % (s, "expert" if mk is None else "nn"))

    ax.set_xlim(-9, 21)
    ax.set_ylim(-6, 6)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectories: expert (dashed) vs NN (solid)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="upper left")
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print("traj ->", out)


if __name__ == "__main__":
    pkg = os.path.join(HERE, "..")
    outdir = os.path.join(pkg, "results")
    os.makedirs(outdir, exist_ok=True)
    loss_curve(os.path.join(pkg, "models", "dock_loss.npy"), os.path.join(outdir, "loss.png"))
    trajectories(os.path.join(pkg, "models"), os.path.join(outdir, "traj.png"))
