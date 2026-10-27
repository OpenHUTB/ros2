#!/usr/bin/env python3
"""Aggregate the per-scene summary CSV files and draw the evaluation figure."""
import argparse
import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

FIELDS = [
    "world", "controller", "samples", "duration_s",
    "rmse_v", "rmse_w", "mae_v", "mae_w", "mean_abs_slip",
    "distance_m", "max_roll_deg", "max_pitch_deg",
    "latency_mean_ms", "latency_p95_ms", "success",
]
CONTROLLERS = ["raw", "pi", "mlp"]
WORLD_ORDER = ["flat", "slope", "low_friction"]
WORLD_LABEL = {"flat": "平地", "slope": "10°斜坡", "low_friction": "低摩擦"}


def load_rows(paths):
    rows = []
    for path in paths:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            for raw in csv.DictReader(handle):
                if raw.get("controller") not in CONTROLLERS:
                    continue
                item = {}
                for key in FIELDS:
                    value = raw.get(key, 0.0)
                    if key in ("world", "controller"):
                        item[key] = value
                    elif key == "success":
                        item[key] = int(float(value))
                    elif key == "samples":
                        item[key] = int(float(value))
                    else:
                        item[key] = float(value)
                rows.append(item)
    return rows


def sort_rows(rows):
    def key(item):
        wi = WORLD_ORDER.index(item["world"]) if item["world"] in WORLD_ORDER else 99
        ci = CONTROLLERS.index(item["controller"]) if item["controller"] in CONTROLLERS else 99
        return (wi, ci)
    return sorted(rows, key=key)


def markdown_table(rows):
    header = ("| 场景 | 控制器 | RMSE_v (m/s) | RMSE_w (rad/s) | "
              "平均打滑 (m/s) | 距离 (m) | 最大侧倾 (°) | 推理耗时 (ms) | 成功 |")
    sep = "|---|---|---|---|---|---|---|---|---|"
    lines = [header, sep]
    for item in rows:
        lines.append("| {} | {} | {:.3f} | {:.3f} | {:.3f} | {:.2f} | {:.1f} | {:.3f} | {} |".format(
            WORLD_LABEL.get(item["world"], item["world"]),
            item["controller"].upper(),
            item["rmse_v"], item["rmse_w"], item["mean_abs_slip"],
            item["distance_m"], item["max_roll_deg"],
            item["latency_mean_ms"], "是" if item["success"] else "否",
        ))
    return "\n".join(lines)


def plot(rows, output):
    worlds = [w for w in WORLD_ORDER if any(r["world"] == w for r in rows)]
    metrics = [
        ("rmse_v", "速度跟踪 RMSE (m/s)"),
        ("rmse_w", "角速度跟踪 RMSE (rad/s)"),
        ("mean_abs_slip", "平均打滑量 (m/s)"),
    ]
    colors = {"raw": "#9aa0a6", "pi": "#1a73e8", "mlp": "#f29900"}
    fig, axes = plt.subplots(1, len(metrics), figsize=(13.5, 4.2))
    if len(metrics) == 1:
        axes = [axes]
    width = 0.26
    x = np.arange(len(worlds))
    for ax, (metric, title) in zip(axes, metrics):
        for offset, controller in zip((-width, 0.0, width), CONTROLLERS):
            values = []
            for world in worlds:
                match = [r for r in rows if r["world"] == world and r["controller"] == controller]
                values.append(match[0][metric] if match else 0.0)
            ax.bar(x + offset, values, width=width, label=controller.upper(),
                   color=colors[controller], edgecolor="white", linewidth=0.6)
        ax.set_xticks(x)
        ax.set_xticklabels([WORLD_LABEL.get(w, w) for w in worlds])
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25, linestyle="--")
        ax.set_axisbelow(True)
    axes[0].legend(frameon=False, ncol=3, loc="upper right")
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    fig.savefig(output, dpi=180)
    print("[evaluate] 图已保存: {}".format(output))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", nargs="*", default=[])
    parser.add_argument("--output-csv", default="data/summary_all.csv")
    parser.add_argument("--output-md", default="data/summary_table.md")
    parser.add_argument("--output-fig", default="data/metrics.png")
    args = parser.parse_args()

    paths = args.input or sorted(glob.glob("data/summary_*.csv"))
    paths = [p for p in paths if os.path.isfile(p) and not p.endswith("summary_all.csv")]
    if not paths:
        raise SystemExit("未找到 summary_*.csv，请先运行 run_benchmark.sh")
    rows = sort_rows(load_rows(paths))
    if not rows:
        raise SystemExit("summary CSV 中没有有效行")

    os.makedirs(os.path.dirname(os.path.abspath(args.output_csv)), exist_ok=True)
    with open(args.output_csv, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    table = markdown_table(rows)
    with open(args.output_md, "w", encoding="utf-8") as handle:
        handle.write(table + "\n")
    print(table)
    plot(rows, args.output_fig)


if __name__ == "__main__":
    main()
