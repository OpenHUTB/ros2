#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成文档用的两张图：故障对比动图 + 高度退化曲线。

用法（在包根目录或任意位置）::

    python tools/gen_figures.py                # 生成到 docs/img/air/uav_actuator_fault_sim/
    python tools/gen_figures.py --out-dir DIR  # 指定输出目录

需要 Pillow（动图）与 matplotlib（曲线）。缺其中一个会跳过对应产物并给出提示，
不会让整个脚本失败——这样在最小依赖的环境里也能跑。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_PKG_ROOT = _HERE.parent                       # src/air/uav_actuator_fault_sim
_REPO_ROOT = _HERE.parents[3]                   # 仓库根：tools → 包 → air → src → 仓库根
sys.path.insert(0, str(_PKG_ROOT))

from uav_actuator_fault_sim.render import make_comparison_gif   # noqa: E402
from uav_actuator_fault_sim.runner import RunConfig, run_many    # noqa: E402

#: 图上要画的故障（顺序即图例顺序）
PLOT_SCENARIOS = ("healthy", "partial_degradation", "stuck", "single_loss", "progressive")


def _altitude_figure(results, out_path: Path) -> bool:
    """画"高度随时间"与"最低高度"两联图。"""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("跳过曲线图：未安装 matplotlib（pip install matplotlib）")
        return False

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.2, 3.4), dpi=170)

    styles = {
        "healthy": ("#2e8b57", "-"),
        "partial_degradation": ("#1f77b4", "-"),
        "stuck": ("#ff7f0e", "--"),
        "single_loss": ("#d62728", "-"),
        "progressive": ("#9467bd", "-."),
    }
    labels = {
        "healthy": "healthy",
        "partial_degradation": "degradation (s=0.4)",
        "stuck": "stuck (s=0.6)",
        "single_loss": "single rotor lost",
        "progressive": "progressive (0.25/s)",
    }
    for r in results:
        color, ls = styles.get(r.scenario, ("0.5", "-"))
        ax1.plot(r.t, r.pos[:, 2], color=color, linestyle=ls, linewidth=1.5,
                 label=labels.get(r.scenario, r.scenario))

    ax1.axhline(0.0, color="0.35", linewidth=0.9, linestyle=":")
    ax1.axvline(2.0, color="0.6", linewidth=0.8, linestyle=":")
    ax1.annotate("fault injected", xy=(2.0, 1.2), xytext=(2.2, 1.9), fontsize=7,
                 color="0.35", arrowprops=dict(arrowstyle="->", color="0.6", lw=0.7))
    # 坠机后高度会掉到 −100 m 量级，截断纵轴才能看清前 1 s 的姿态
    ax1.set_ylim(-6.0, 2.0)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("altitude (m)")
    ax1.set_title("Altitude response to rotor faults", fontsize=9)
    ax1.grid(alpha=0.25)
    ax1.legend(fontsize=6.5, loc="lower left")

    order = [r.scenario for r in results]
    mins = [r.min_altitude for r in results]
    colors = [styles.get(s, ("0.5", "-"))[0] for s in order]
    ax2.barh(range(len(order)), mins, color=colors, height=0.6)
    ax2.set_yticks(range(len(order)), [labels.get(s, s) for s in order], fontsize=7)
    ax2.axvline(0.0, color="0.35", linewidth=0.9)
    ax2.set_xlabel("minimum altitude (m)")
    ax2.set_title("Minimum altitude (clipped at −120 m)", fontsize=9)
    ax2.set_xlim(-125.0, 16.0)
    ax2.grid(alpha=0.25, axis="x")
    for i, v in enumerate(mins):
        # 正值（健康/衰减）要右留白，否则标签会被坐标轴裁掉
        ax2.text(v + 2.0 if v < 0 else v + 2.0, i, f"{v:.1f}", va="center", fontsize=6.5)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", default=str(_REPO_ROOT / "docs" / "img"
                                            / "air" / "uav_actuator_fault_sim"),
                    help="产物输出目录")
    ap.add_argument("--seconds", type=float, default=8.0, help="曲线用的仿真时长")
    ap.add_argument("--gif-seconds", type=float, default=6.0, help="动图时长")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=240)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--skip-gif", action="store_true")
    args = ap.parse_args(argv)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    from uav_actuator_fault_sim.scenario import make_case

    cases = [make_case("open_ground", name, seed=0, rotor=0, onset=2.0)
             for name in PLOT_SCENARIOS]
    results = run_many(cases, cfg=RunConfig(duration=args.seconds))

    png = out_dir / "altitude_response.png"
    if _altitude_figure(results, png):
        print(f"曲线图 → {png}")

    if not args.skip_gif:
        gif = out_dir / "fault_comparison.gif"
        make_comparison_gif(str(gif), scene="open_ground", seconds=args.gif_seconds,
                            width=args.width, height=args.height, fps=args.fps,
                            camera="track")
        size_mb = os.path.getsize(gif) / (1024 * 1024)
        print(f"对比动图 → {gif}（{size_mb:.2f} MB）")
        if size_mb > 5.0:
            print("警告：超过作业建议的 5 MB，请降低 --width/--fps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
