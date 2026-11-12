"""对比实验：三种控制器 × 三档感知噪声 × 两个种子。"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .env import AckermannEnv
from .nav import run_episode

CONTROLLERS = ("pp", "mppi", "nn")
NOISE = (0.0, 1.0, 2.0)


def sweep(env: AckermannEnv, net=None, controllers=CONTROLLERS, noises=NOISE,
          seeds=(0, 1), log=print) -> list[dict]:
    runs = []
    for ctrl in controllers:
        for noise in noises:
            for seed in seeds:
                r = run_episode(env, controller=ctrl, noise=noise, seed=seed, net=net)
                runs.append({"controller": ctrl, "noise": noise, "seed": seed,
                             "rms": r.rms, "max_err": r.max_err, "completed": r.completed,
                             "steps": r.steps, "mean_ms": r.mean_ms})
                log(f"[{ctrl:4s} noise={noise:.1f} seed={seed}] "
                    f"RMS={r.rms:.3f} m  max={r.max_err:.3f} m  完成={r.completed}  "
                    f"步数={r.steps}  耗时={r.mean_ms:.2f} ms/步")
    return runs


def aggregate(runs: list[dict]) -> dict:
    out = {}
    for ctrl in sorted({r["controller"] for r in runs}):
        for noise in sorted({r["noise"] for r in runs}):
            sel = [r for r in runs if r["controller"] == ctrl and r["noise"] == noise]
            if not sel:
                continue
            out[f"{ctrl}@{noise:.1f}"] = {
                "rms_mean": round(float(np.mean([r["rms"] for r in sel])), 3),
                "rms_max": round(float(np.max([r["rms"] for r in sel])), 3),
                "completion": round(float(np.mean([r["completed"] for r in sel])), 3),
                "ms_mean": round(float(np.mean([r["mean_ms"] for r in sel])), 3),
                "runs": len(sel),
            }
    return out


def plot_sweep(runs: list[dict], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    noises = sorted({r["noise"] for r in runs})
    colors = {"pp": "#9b51e0", "mppi": "#2f80ed", "nn": "#27ae60"}
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ctrl in CONTROLLERS:
        rms = [np.mean([r["rms"] for r in runs if r["controller"] == ctrl and r["noise"] == n])
               for n in noises]
        ok = [np.mean([r["completed"] for r in runs
                       if r["controller"] == ctrl and r["noise"] == n])
              for n in noises]
        axes[0].plot(noises, rms, "o-", color=colors[ctrl], label=ctrl.upper())
        axes[1].plot(noises, ok, "s--", color=colors[ctrl], label=ctrl.upper())
    axes[0].set_xlabel("perception noise level")
    axes[0].set_ylabel("lateral RMS (m)")
    axes[1].set_xlabel("perception noise level")
    axes[1].set_ylabel("completion rate")
    axes[1].set_ylim(-0.05, 1.05)
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("tracking accuracy vs perception noise: MPPI / distilled NN / pure pursuit")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_track(trace, path_ref, out: Path, title: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(path_ref[:, 0], path_ref[:, 1], "-", color="#888", linewidth=1.0, label="reference")
    t = np.array([p[0] for p in trace])
    ax.plot(t[:, 0], t[:, 1], "-", color="#2f80ed", linewidth=1.2, label="vehicle")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(fontsize=8)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
