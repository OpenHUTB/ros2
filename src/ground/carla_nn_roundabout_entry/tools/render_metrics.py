"""Render comparison charts from deterministic roundabout rollouts."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from carla_nn_roundabout_entry.roundabout_sim import run_headless_with_controller


SEEDS = (3, 7, 11, 19, 23)
CONTROLLERS = (("gap-policy", "间隙策略", "#047857"),
               ("baseline", "固定油门基线", "#d97706"))


def collect_metrics() -> list[dict[str, float | int | str]]:
    rows = []
    for controller, label, _ in CONTROLLERS:
        for seed in SEEDS:
            result = run_headless_with_controller(seed, 360, controller)
            rows.append({
                "controller": controller,
                "label": label,
                "seed": seed,
                "collision_frames": int(result["collision_frames"]),
                "minimum_clearance_m": float(result["minimum_clearance_m"]),
            })
    return rows


def render_chart(rows: list[dict[str, float | int | str]], output: Path) -> None:
    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    if font_path.exists():
        from matplotlib.font_manager import FontProperties

        font = FontProperties(fname=str(font_path))
        plt.rcParams["font.family"] = font.get_name()
    else:
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    figure, axes = plt.subplots(1, 2, figsize=(11, 4.8), constrained_layout=True)
    seeds = np.asarray(SEEDS)
    bar_width = 0.34
    offsets = (-bar_width / 2, bar_width / 2)

    for offset, (controller, label, color) in zip(offsets, CONTROLLERS):
        subset = [row for row in rows if row["controller"] == controller]
        collision = [int(row["collision_frames"]) for row in subset]
        clearance = [float(row["minimum_clearance_m"]) for row in subset]
        axes[0].bar(seeds + offset, collision, width=bar_width,
                    label=label, color=color, alpha=0.9)
        axes[1].plot(seeds, clearance, marker="o", linewidth=2,
                     label=label, color=color)

    axes[0].set_title("各固定种子的碰撞帧数")
    axes[0].set_xlabel("随机种子")
    axes[0].set_ylabel("碰撞帧")
    axes[0].set_xticks(seeds)
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend(frameon=False)

    axes[1].axhline(0.0, color="#6b7280", linewidth=1, linestyle="--")
    axes[1].set_title("各固定种子的最小净间距")
    axes[1].set_xlabel("随机种子")
    axes[1].set_ylabel("最小净间距（米）")
    axes[1].set_xticks(seeds)
    axes[1].grid(alpha=0.25)
    axes[1].legend(frameon=False)

    figure.suptitle("环岛策略对照｜每个种子 360 步", fontsize=14)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=170, facecolor="white")
    plt.close(figure)


def main() -> None:
    repository_root = Path(__file__).resolve().parents[4]
    rows = collect_metrics()
    output = repository_root / "docs" / "img" / "roundabout_metrics.png"
    render_chart(rows, output)
    summary = {}
    for controller, label, _ in CONTROLLERS:
        subset = [row for row in rows if row["controller"] == controller]
        summary[label] = {
            "scenarios_with_collisions": sum(
                int(row["collision_frames"]) > 0 for row in subset
            ),
            "mean_collision_frames": round(float(np.mean(
                [row["collision_frames"] for row in subset]
            )), 2),
            "worst_minimum_clearance_m": round(min(
                float(row["minimum_clearance_m"]) for row in subset
            ), 3),
        }
    print(json.dumps({"seeds": SEEDS, "steps": 360, "results": summary},
                     ensure_ascii=False, indent=2))
    print(f"图表已生成：{output}")


if __name__ == "__main__":
    main()
