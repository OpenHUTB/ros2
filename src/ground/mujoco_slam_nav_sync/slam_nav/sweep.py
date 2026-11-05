"""对照实验：定位漂移 × 是否启用 SLAM × 随机种子。

这就是"导航与 SLAM 同步仿真"的核心：同一个导航栈，只改定位来源，
看成功率 / ATE / 路径长度怎么变。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .env import DiffDriveEnv
from .nav import NavConfig, run_episode

SLIPS = (0.0, 0.05, 0.10, 0.20)


def sweep(env: DiffDriveEnv, net=None, slips=SLIPS, seeds=(0, 1, 2, 3, 4), scene: str = "maze",
          cfg: NavConfig | None = None, log=print) -> list[dict]:
    cfg = cfg or NavConfig()
    records = []
    for slip in slips:
        for use_slam in (True, False):
            for seed in seeds:
                res = run_episode(env, net=net, use_slam=use_slam, slip=slip,
                                  seed=seed, scene=scene, cfg=cfg)
                rec = {
                    "scene": scene, "mode": res.mode, "slip": slip, "seed": seed,
                    "success": res.success, "crashed": res.crashed, "steps": res.steps,
                    "ate_rmse": res.ate_rmse, "final_dist": res.final_dist,
                    "path_length": res.path_length, "map_known": res.map_known,
                }
                records.append(rec)
                log(f"[{res.mode:14s} slip={slip:.2f} seed={seed}] "
                    f"成功={str(res.success):5s} 碰撞={str(res.crashed):5s} "
                    f"ATE={res.ate_rmse:.3f} m 终点误差={res.final_dist:.2f} m "
                    f"路径={res.path_length:.2f} m")
    return records


def aggregate(records: list[dict]) -> dict:
    """按 (mode, slip) 聚合成功率和 ATE 均值。"""
    table = {}
    for rec in records:
        key = (rec["mode"], rec["slip"])
        table.setdefault(key, []).append(rec)
    out = {}
    for (mode, slip), rows in sorted(table.items()):
        out[f"{mode}@{slip:.2f}"] = {
            "success_rate": round(float(np.mean([r["success"] for r in rows])), 3),
            "crashes": int(sum(r["crashed"] for r in rows)),
            "ate_mean": round(float(np.mean([r["ate_rmse"] for r in rows])), 3),
            "ate_max": round(float(np.max([r["ate_rmse"] for r in rows])), 3),
            "path_mean": round(float(np.mean([r["path_length"] for r in rows])), 2),
            "runs": len(rows),
        }
    return out


def plot_sweep(records: list[dict], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    slips = sorted({r["slip"] for r in records})
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for mode, color, marker in (("slam", "#2f80ed", "o-"), ("dead-reckoning", "#eb5757", "s--")):
        succ = [np.mean([r["success"] for r in records if r["mode"] == mode and r["slip"] == s])
                for s in slips]
        ate = [np.mean([r["ate_rmse"] for r in records if r["mode"] == mode and r["slip"] == s])
               for s in slips]
        axes[0].plot([s * 100 for s in slips], succ, marker, color=color, label=mode)
        axes[1].plot([s * 100 for s in slips], ate, marker, color=color, label=mode)
    axes[0].set_xlabel("wheel slip (%)")
    axes[0].set_ylabel("navigation success rate")
    axes[0].set_ylim(-0.05, 1.05)
    axes[1].set_xlabel("wheel slip (%)")
    axes[1].set_ylabel("ATE RMSE (m)")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("navigation vs localisation drift: EKF-SLAM vs dead reckoning")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_map(grid, truth, path: Path, traj=None, goal=None) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(grid.logodds, origin="lower", cmap="Greys", vmin=-3, vmax=3,
              extent=(-1, 13, -7, 7))
    ax.imshow(np.where(truth, 1.0, np.nan), origin="lower", cmap="Reds", alpha=0.35,
              extent=(-1, 13, -7, 7))
    if traj is not None and len(traj):
        t = np.array(traj)
        ax.plot(t[:, 0], t[:, 1], "-", color="#2f80ed", linewidth=1.2, label="truth")
        ax.plot(t[:, 2], t[:, 3], "--", color="#eb5757", linewidth=1.0, label="SLAM")
        ax.legend(fontsize=8)
    if goal is not None:
        ax.plot(goal[0], goal[1], "*", color="#f2c94c", markersize=14)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("occupancy map built online from lidar")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
