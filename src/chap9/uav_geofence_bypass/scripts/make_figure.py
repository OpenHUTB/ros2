#!/usr/bin/env python3
"""禁飞区绕行图：直接复用 core 的膨胀算法与 config 里的场景，避免图与代码不一致。

用法：
    python3 scripts/make_figure.py --csv /tmp/uav_geofence_bypass.csv --out /tmp/uav_geofence_bypass.png
"""

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon as MplPolygon  # noqa: E402

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_ROOT)

from uav_geofence_bypass import core  # noqa: E402


def load_scene(path):
    """从 config/params.yaml 读取场景；缺文件时回退到 core 里的默认值。"""
    scene = {
        "zones": [list(z) for z in core.ZONES],
        "margin": 0.6,
        "start": core.START,
        "goal": core.GOAL,
    }
    if not path or not os.path.exists(path):
        return scene
    try:
        import yaml
    except ImportError:
        return scene
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    params = data.get("uav_geofence_bypass", {}).get("ros__parameters", {}) or {}
    if "zones" in params:
        scene["zones"] = [
            [(float(v[i]), float(v[i + 1])) for i in range(0, len(v), 2)] for v in params["zones"]
        ]
    scene["margin"] = float(params.get("margin", scene["margin"]))
    if "start" in params:
        scene["start"] = (float(params["start"][0]), float(params["start"][1]))
    if "goal" in params:
        scene["goal"] = (float(params["goal"][0]), float(params["goal"][1]))
    return scene


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="/tmp/uav_geofence_bypass.csv")
    ap.add_argument("--params", default=os.path.join(PKG_ROOT, "config", "params.yaml"))
    ap.add_argument("--out", default="/tmp/uav_geofence_bypass.png")
    args = ap.parse_args()

    scene = load_scene(args.params)
    rows = []
    with open(args.csv, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})

    fig, ax = plt.subplots(figsize=(9, 5.2))
    for idx, zone in enumerate(scene["zones"]):
        ax.add_patch(MplPolygon(zone, closed=True, fc="#ffcccc", ec="#cc0000", alpha=0.7,
                                label="no-fly zone" if idx == 0 else None))
        # 用 core 的真实膨胀结果，而不是质心径向放大的近似
        grown = core.inflate_polygon(zone, scene["margin"])
        ax.add_patch(MplPolygon(grown, closed=True, fill=False, ec="#cc0000", ls="--", lw=1.2,
                                label="inflated %.1f m" % scene["margin"] if idx == 0 else None))

    start, goal = scene["start"], scene["goal"]
    ax.plot([start[0], goal[0]], [start[1], goal[1]], "r:", lw=1.5, label="direct line")
    if rows:
        ax.plot([r["x"] for r in rows], [r["y"] for r in rows], "#1f77b4", lw=2, label="flown path")
        ax.scatter([rows[0]["x"]], [rows[0]["y"]], c="green", s=50, label="start")
        ax.scatter([rows[-1]["x"]], [rows[-1]["y"]], c="orange", s=40, label="last sample")
    ax.scatter([goal[0]], [goal[1]], c="blue", marker="*", s=120, label="goal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("geofence inflation (core.inflate_polygon) and flown path")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="upper left")
    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    last = rows[-1] if rows else {}
    print("saved %s rows=%d last=(%.2f, %.2f)" % (args.out, len(rows), last.get("x", 0.0), last.get("y", 0.0)))


if __name__ == "__main__":
    main()
