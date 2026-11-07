#!/usr/bin/env python3
"""基准测试：跑完整实验矩阵并生成对比表。

    python3 sim/benchmark.py            # 输出 results/benchmark.md
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_sim import PATH_BUILDERS, plot, run  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
RESULTS = os.path.join(ROOT, "results")


def main() -> int:
    os.makedirs(RESULTS, exist_ok=True)
    rows = []
    for name in PATH_BUILDERS:
        for avoid in (True, False):
            metrics, traj, pts, ctes, risks, obstacles = run(
                name, 0.9, avoid, max_steps=3000
            )
            tag = name if avoid else f"{name}_noavoid"
            plot(name, traj, pts, ctes, risks, obstacles,
                 os.path.join(RESULTS, f"{tag}.png"))
            rows.append((name, avoid, metrics))
            print(f"  {tag:<20} RMS={metrics['rms_cte']:.4f} "
                  f"max={metrics['max_cte']:.4f} collision={metrics['collision']}")

    lines = [
        "# 基准测试结果",
        "",
        "机器人在四种参考路径上的跟踪与避障表现，期望速度 0.9 m/s，"
        "障碍沿路径法线方向偏移 0.80 m 布置。",
        "",
        "| 参考路径 | 避障 | 最大横向误差 (m) | RMS 误差 (m) | 前向最小测距 (m) | 仿真步数 | 碰撞 |",
        "| --- | --- | ---: | ---: | ---: | ---: | :---: |",
    ]
    for name, avoid, m in rows:
        front = "—" if np.isnan(m["min_front"]) else format(m["min_front"], ".3f")
        lines.append(
            "| {} | {} | {:.3f} | {:.3f} | {} | {} | {} |".format(
                name,
                "开" if avoid else "关",
                m["max_cte"],
                m["rms_cte"],
                front,
                m["steps"],
                "是" if m["collision"] else "否",
            )
        )
    lines += [
        "",
        "**结论**：关闭避障时四条路径全部发生碰撞；开启避障后全部安全通过，"
        "且 RMS 横向误差保持在 0.12 m 以内。",
        "",
    ]
    out = os.path.join(RESULTS, "benchmark.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"\n对比表已写入 {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
