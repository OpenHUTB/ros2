#!/usr/bin/env python3
"""曲线图：轨迹（叠地形等高线 + 车体速度着色）/ IMU 姿态 / 四悬架行程。

physics_only_demo.py 与 headless_demo.py 共用这一份，保证文档页的图注和仓库里那张图
是同一版式（纯物理出图与无头渲染出图不再各画一套）。
"""
import os

import numpy as np


def make_curves(samples, target, out_path):
    """把采样序列画成三联图并写盘，返回输出路径。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from terrain import elevation

    t = np.array([s["t"] for s in samples])
    x = np.array([s["x"] for s in samples])
    y = np.array([s["y"] for s in samples])
    roll = np.degrees([s["roll"] for s in samples])
    pitch = np.degrees([s["pitch"] for s in samples])
    v = np.array([s["v_body"] for s in samples])
    sus = np.array([s["sus"] for s in samples]) * 1000  # mm

    # colorbar 与 tight_layout 冲突（会刷 UserWarning: Axes not compatible with tight_layout），
    # 改用 constrained_layout 由 matplotlib 统一排版
    fig = plt.figure(figsize=(12, 8.2), dpi=110, constrained_layout=True)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.35, 1])

    # ---- 轨迹 + 地形等高线 + 速度着色
    ax = fig.add_subplot(gs[0, :])
    gx, gy = np.meshgrid(np.linspace(x.min() - 4, x.max() + 3, 260),
                         np.linspace(y.min() - 3.2, y.max() + 3.2, 200))
    gz = elevation(gx, gy)
    cf = ax.contourf(gx, gy, gz, levels=22, cmap="YlOrBr", alpha=0.9)
    ax.contour(gx, gy, gz, levels=12, colors="k", linewidths=0.35, alpha=0.35)
    ax.plot(x, y, lw=2.4, color="#0a6cff", label="vehicle path")
    ax.scatter([x[0]], [y[0]], s=45, color="w", edgecolor="k", zorder=5, label="start")
    ax.scatter([target[0]], [target[1]], s=110, marker="*", color="#e03030",
               edgecolor="k", zorder=6, label="target")
    sc = ax.scatter(x[::4], y[::4], c=v[::4], s=9, cmap="cool", zorder=4)
    cb = fig.colorbar(sc, ax=ax, pad=0.01, fraction=0.03)
    cb.set_label("body speed [m/s]")
    fig.colorbar(cf, ax=ax, pad=0.02, fraction=0.03).set_label("terrain elevation [m]")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Path over undulating hfield terrain (480x480, cell 0.25 m)")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.2)

    # ---- IMU 姿态
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, pitch, color="#d1495b", label="pitch")
    ax2.plot(t, roll, color="#2e86ab", label="roll")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("attitude [deg]")
    ax2.set_title("IMU attitude (continuous)")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.25)

    # ---- 四悬架行程
    ax3 = fig.add_subplot(gs[1, 1])
    for i, nm in enumerate(["FL", "FR", "RL", "RR"]):
        ax3.plot(t, sus[:, i], lw=1.4, label=nm)
    ax3.set_xlabel("time [s]")
    ax3.set_ylabel("suspension travel [mm]")
    ax3.set_title("4 independent suspensions")
    ax3.legend(fontsize=8, ncol=4)
    ax3.grid(alpha=0.25)

    fig.suptitle("MuJoCo off-road robot: independent suspension + IMU feedback",
                 fontsize=13)
    fig.savefig(out_path)
    plt.close(fig)
    print(f"curves: {out_path}")
    return os.path.abspath(out_path)
