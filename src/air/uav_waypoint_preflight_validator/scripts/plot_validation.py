#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绘制一个或多个任务的俯视校验结果。"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from preflight_validator import validate_mission_file


def _plot(ax, report, title):
    for segment in report["segments"]:
        color = "#28a745" if segment["status"] == "VALID" else "#dc3545"
        ax.plot(
            [segment["start"][0], segment["end"][0]],
            [segment["start"][1], segment["end"][1]],
            color=color,
            linewidth=2.5,
        )
    points = [item for item in report["waypoints"] if "point" in item]
    if points:
        colors = []
        for item in points:
            if item["status"] == "VALID":
                colors.append("#28a745")
            elif item["status"] == "WARNING":
                colors.append("#ffc107")
            else:
                colors.append("#dc3545")
        ax.scatter(
            [item["point"][0] for item in points],
            [item["point"][1] for item in points],
            c=colors,
            s=45,
            zorder=3,
        )
        offsets = ((8, 8), (8, -20), (-52, 10), (-52, -24))
        for item in points:
            ax.annotate(
                "WP%d\n%.1fm" % (item["index"], item["point"][2]),
                (item["point"][0], item["point"][1]),
                xytext=offsets[item["index"] % len(offsets)],
                textcoords="offset points",
                fontsize=8,
            )
    ax.scatter([0], [0], marker="x", s=80, color="black", label="takeoff frame")
    ax.set_title("%s | %s" % (title, report["status"]))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    summary = report["summary"]
    detail = "dist %.2f m, ETA %.2f s, errors %d, warnings %d" % (
        summary["total_distance_m"],
        summary["estimated_duration_s"],
        summary["error_count"],
        summary["warning_count"],
    )
    issues = report["errors"] + report["warnings"]
    if issues:
        detail += "\n" + "; ".join(item["code"] for item in issues[:4])
    ax.text(
        0.02,
        0.97,
        detail,
        transform=ax.transAxes,
        fontsize=8,
        va="top",
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="#cccccc", alpha=0.9),
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description="绘制航点预检结果")
    parser.add_argument("--valid", required=True, help="合法任务 YAML")
    parser.add_argument("--invalid", required=True, help="非法任务 YAML")
    parser.add_argument("--output", required=True, help="PNG 输出路径")
    args = parser.parse_args(argv)

    reports = [
        validate_mission_file(args.valid),
        validate_mission_file(args.invalid),
    ]
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.8), dpi=120)
    _plot(axes[0], reports[0], "valid mission")
    _plot(axes[1], reports[1], "invalid mission")
    figure.tight_layout()
    figure.savefig(args.output)
    print(args.output)
    return 0


if __name__ == "__main__":
    main()
