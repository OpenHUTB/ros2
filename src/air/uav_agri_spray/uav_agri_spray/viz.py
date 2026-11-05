#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Matplotlib outputs: deposition heatmap with route overlay (English titles)."""
from __future__ import annotations

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

# B 套配色：tab10 + 虚线主色，dpi=110，图例 lower right
MAIN_COLOR = plt.get_cmap("tab10")(0)
SECOND_COLOR = plt.get_cmap("tab10")(3)
DPI = 110


def map_payload(grid, route: np.ndarray) -> dict:
    """Serialize a DepositMap + route for the results JSON."""
    return {
        "x0": grid.x0, "y0": grid.y0, "resolution": grid.resolution,
        "nx": int(grid.deposit.shape[1]), "ny": int(grid.deposit.shape[0]),
        "deposit": np.round(grid.deposit, 3).tolist(),
        "route": np.round(np.asarray(route), 2).tolist(),
    }


def plot_map(mp: dict, out_png: str, title: str, route2=None,
             label1: str = "planned route", label2: str = "flown route") -> str:
    """Heatmap of deposition with route overlay; English title, lower-right legend."""
    dep = np.asarray(mp["deposit"], dtype=float)
    res = mp["resolution"]
    ext = [mp["x0"], mp["x0"] + dep.shape[1] * res,
           mp["y0"], mp["y0"] + dep.shape[0] * res]
    fig, ax = plt.subplots(figsize=(8, 5.5))
    im = ax.imshow(dep, origin="lower", extent=ext, cmap="viridis",
                   vmin=0.0, vmax=max(1.5, float(dep.max())))
    fig.colorbar(im, ax=ax, label="relative deposition")
    route = np.asarray(mp.get("route") or [], dtype=float)
    if route.size:
        ax.plot(route[:, 0], route[:, 1], "--", color=MAIN_COLOR, lw=1.3,
                label=label1)
    if route2 is not None:
        r2 = np.asarray(route2, dtype=float)
        ax.plot(r2[:, 0], r2[:, 1], "-", color=SECOND_COLOR, lw=0.8,
                alpha=0.75, label=label2)
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="lower right")
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=DPI)
    plt.close(fig)
    return out_png


def write_maps(report: dict, out_dir: str) -> list:
    """Write per-case maps from the results JSON; first case is the headline map."""
    outs = []
    for i, m in enumerate(report.get("missions", [])):
        mp = m.get("coverage_map")
        if not mp:
            continue
        name = "coverage_map.png" if i == 0 else \
            "coverage_map_%s.png" % m["case"]
        outs.append(plot_map(mp, os.path.join(out_dir, name),
                             "Deposition coverage - %s" % m["case"]))
    cl = report.get("closed_loop")
    if cl and cl.get("actual_map"):
        mp = dict(cl["actual_map"])
        outs.append(plot_map(
            mp, os.path.join(out_dir, "coverage_map_actual.png"),
            "Actual deposition from MuJoCo flight - %s" % cl["case"]))
    return outs
