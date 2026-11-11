#!/usr/bin/env python3
"""画指标曲线：指令速度 vs 实际速度、骨盆高度，用于文档里的性能评价。

    python scripts/plot_metrics.py --out docs/img/humanoid/humanoid_h1_walk_metrics.png

标签用英文，避免服务器缺中文字体时出现方块。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_h1_walk import ROUTINE, Walker, load_specs, routine_command  # noqa: E402


def record(robot: str, specs) -> dict:
    walker = Walker(robot, specs)
    total = sum(seconds for _, seconds, _, _ in ROUTINE)
    t, v_cmd, v_act, h, wz_cmd, wz_act = [], [], [], [], [], []
    while walker.data.time < total:
        cmd, _ = routine_command(walker.data.time)
        walker.step(cmd)
        t.append(float(walker.data.time))
        v_cmd.append(float(cmd[0]))
        v_act.append(float(walker.base_lin_vel_body()[0]))
        h.append(walker.base_height)
        wz_cmd.append(float(cmd[2]))
        wz_act.append(float(walker.data.qvel[5]))
    series = {
        "t": t,
        "v_cmd": v_cmd,
        "v_act": v_act,
        "h": h,
        "wz_cmd": wz_cmd,
        "wz_act": wz_act,
    }
    return {key: np.asarray(value) for key, value in series.items()}


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--out", type=Path, default=Path("metrics.png"))
    args = parser.parse_args()

    specs = load_specs()
    data = {robot: record(robot, specs) for robot in specs}

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True,
                             gridspec_kw={"height_ratios": [2, 1]})
    colors = {"h1": "#c0392b", "h1_2": "#2c6fbb"}

    ax = axes[0]
    first = next(iter(data.values()))
    ax.plot(first["t"], first["v_cmd"], "k--", lw=1.0, label="command")
    for robot, d in data.items():
        ax.plot(d["t"], d["v_act"], color=colors.get(robot, None), lw=1.2, label=f"{robot} actual")
    ax.set_ylabel(r"forward velocity $v_x$ (m/s)")
    ax.axhline(0, color="0.7", lw=0.6)
    ax.legend(loc="lower left", fontsize=8, ncol=3, frameon=False)
    ax.set_title("Velocity tracking over the fixed routine", fontsize=10)

    ax = axes[1]
    for robot, d in data.items():
        ax.plot(d["t"], d["h"], color=colors.get(robot, None), lw=1.2, label=robot)
    ax.set_ylabel("pelvis height (m)")
    ax.set_xlabel("simulation time (s)")
    ax.set_ylim(0.6, 1.15)
    ax.axhline(0.5, color="#b03a2e", ls=":", lw=1.0)
    ax.text(0.4, 0.575, "fall threshold 0.5 m", color="#b03a2e", fontsize=7)
    ax.legend(loc="lower right", fontsize=8, frameon=False)

    for a in axes:
        a.grid(alpha=0.25)
        for spine in ("top", "right"):
            a.spines[spine].set_visible(False)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(f"已保存 {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
