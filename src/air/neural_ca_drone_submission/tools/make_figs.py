#!/usr/bin/env python3
"""Generate the figures used by the documentation (``docs/assets/figs``).

Everything here is produced from the shipped simulator and weights, so the
figures can always be regenerated with ``python tools/make_figs.py``.
"""

from __future__ import annotations

import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle, Circle  # noqa: E402

from ca_drone.nn.autograd import Tensor  # noqa: E402
from ca_drone.pipeline import NeuralStack, run_episode  # noqa: E402
from ca_drone.render.scene import (compose_frame, colorize_depth,  # noqa: E402
                                   depth_image)
from ca_drone.mapping import GridMap  # noqa: E402
from ca_drone.planning.astar import astar, cells_to_world  # noqa: E402
from ca_drone.models.planner import predict_field  # noqa: E402
from ca_drone.sim.world import Box, Cylinder, make_world  # noqa: E402

OUT = os.path.join(os.path.dirname(__file__), "..", "docs", "assets", "figs")
SCENES = ["sparse", "dense", "dynamic", "corridor", "unseen"]
FONT = "C:/Windows/Fonts/consola.ttf" if os.name == "nt" else "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"


def _f(size: int) -> ImageFont.FreeTypeFont:
    try:
        return ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default()


def scene_snapshots() -> None:
    for name in SCENES:
        world = make_world(name)
        img = compose_frame(world, world.start, np.zeros(3), 0.0, None,
                            {"mode": "SCENE", "progress": 0.0,
                             "subtitle": "scene preview - static frame"})
        img.save(os.path.join(OUT, f"scene_{name}.png"))
        print("scene", name)


def perception_figure(stack: NeuralStack) -> None:
    world = make_world("dense")
    pose = np.array([13.0, 0.2, 3.0])
    depth, ids = depth_image(world, pose, 0.0, 64, 48, 90, 12.0)
    prob = stack.perception(Tensor(np.clip(depth / 12, 0, 1)[None, None])).data[0, 0]
    gt = (ids >= 0) & (depth < 11.5)

    def mask_img(m: np.ndarray) -> Image.Image:
        arr = np.zeros((48, 64, 3), np.uint8)
        arr[..., 0] = (m * 235).astype(np.uint8)
        arr[..., 1] = (m * 120).astype(np.uint8)
        return Image.fromarray(arr).resize((320, 240), Image.NEAREST)

    panels = [colorize_depth(depth, 12.0).resize((320, 240), Image.NEAREST),
              mask_img(gt), mask_img(prob > 0.5)]
    titles = ["RGB-D input (colorized)", "ground-truth occupancy", "PerceptionNet mask"]
    canvas = Image.new("RGB", (3 * 320 + 40, 330), (18, 22, 28))
    d = ImageDraw.Draw(canvas)
    for i, (p, t) in enumerate(zip(panels, titles)):
        x = 10 + i * 330
        canvas.paste(p, (x, 40))
        d.text((x, 14), t, font=_f(13), fill=(210, 220, 235))
    inter = np.logical_and(gt, prob > 0.5).sum()
    union = np.logical_or(gt, prob > 0.5).sum()
    d.text((12, 300), f"IoU on this frame: {inter / max(union, 1):.3f}", font=_f(12),
           fill=(120, 220, 200))
    canvas.save(os.path.join(OUT, "perception_triplet.png"))
    print("perception IoU", inter / max(union, 1))


def planning_figure(stack: NeuralStack) -> None:
    world = make_world("dense")
    gm = GridMap.from_world(world, 0.5, inflate=0.45)
    start = gm.to_cell(world.start)
    goal = gm.to_cell(world.goal)

    yy, xx = np.mgrid[0:gm.occ.shape[0], 0:gm.occ.shape[1]]
    h_classic = 0.5 * np.hypot(xx - goal[0], yy - goal[1])

    t0 = __import__("time").time()
    rc = astar(gm.occ, start, goal, 0.5, heuristic=h_classic)
    tc = (__import__("time").time() - t0) * 1e3
    t0 = __import__("time").time()
    field = predict_field(stack.planner, gm.occ, goal, 0.5)
    rn = astar(gm.occ, start, goal, 0.5, heuristic=field, weight=1.15)
    tn = (__import__("time").time() - t0) * 1e3

    fig, ax = plt.subplots(figsize=(11, 5.2), dpi=150)
    ax.set_facecolor("#12161c")
    fig.patch.set_facecolor("#12161c")
    for o in world.obstacles():
        if isinstance(o, Box):
            ax.add_patch(Rectangle((o.x - o.sx / 2, o.y - o.sy / 2), o.sx, o.sy,
                                   color="#7a8fa6", alpha=0.75))
        elif isinstance(o, Cylinder):
            ax.add_patch(Circle((o.x, o.y), o.radius, color="#8aa0b6", alpha=0.8))
    pc = cells_to_world(rc["path_cells"], gm.x0, gm.y0, 0.5)
    pn = cells_to_world(rn["path_cells"], gm.x0, gm.y0, 0.5)
    ax.plot(pc[:, 0], pc[:, 1], color="#5ad1ff", lw=2.2,
            label=f"classical A* (h=euclid, {rc['expanded']} expand, {tc:.1f} ms)")
    ax.plot(pn[:, 0], pn[:, 1], color="#ffb454", lw=2.0, ls="--",
            label=f"neural A* (h=NN, w=1.15, {rn['expanded']} expand, {tn:.1f} ms)")
    ax.plot(*world.start[:2], "o", color="#59f0b0", ms=9, label="start")
    ax.plot(*world.goal[:2], "*", color="#ff6b6b", ms=15, label="goal")
    ax.set_xlim(0, 52)
    ax.set_ylim(-22, 22)
    ax.set_aspect("equal")
    ax.tick_params(colors="#9fb0c2")
    for s in ax.spines.values():
        s.set_color("#3a4656")
    ax.set_xlabel("x [m]", color="#9fb0c2")
    ax.set_ylabel("y [m]", color="#9fb0c2")
    ax.set_title("Dense scene: classical vs. neural-heuristic A*", color="#e8eef5")
    leg = ax.legend(loc="upper right", fontsize=8, facecolor="#1b222c", edgecolor="#3a4656")
    for t in leg.get_texts():
        t.set_color("#cfdae6")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "planning_compare.png"), facecolor=fig.get_facecolor())
    plt.close(fig)
    print("planning", rc["expanded"], rn["expanded"])


def trajectories(stack: NeuralStack) -> None:
    for scene in SCENES:
        fig, ax = plt.subplots(figsize=(11, 5.0), dpi=150)
        ax.set_facecolor("#12161c")
        fig.patch.set_facecolor("#12161c")
        world = make_world(scene)
        for o in world.obstacles():
            if isinstance(o, Box):
                ax.add_patch(Rectangle((o.x - o.sx / 2, o.y - o.sy / 2), o.sx, o.sy,
                                       color="#7a8fa6", alpha=0.8))
            elif isinstance(o, Cylinder):
                ax.add_patch(Circle((o.x, o.y), o.radius, color="#8aa0b6", alpha=0.85))
        for mode, color, label in [("expert", "#5ad1ff", "expert"),
                                   ("full", "#59f0b0", "full (NN)"),
                                   ("e2e", "#ffb454", "end-to-end")]:
            tr: list = []
            r = run_episode(scene, mode, stack, max_steps=650, render_every=0, trace=tr)
            p = np.asarray(tr)
            ax.plot(p[:, 0], p[:, 1], color=color, lw=1.9,
                    label=f"{label} - {'success' if r.success else ('collision' if r.collision else 'timeout')}")
        ax.plot(*world.start[:2], "o", color="#59f0b0", ms=8)
        ax.plot(*world.goal[:2], "*", color="#ff6b6b", ms=14)
        ax.set_xlim(0, 52)
        ax.set_ylim(-22, 22)
        ax.set_aspect("equal")
        ax.tick_params(colors="#9fb0c2")
        for s in ax.spines.values():
            s.set_color("#3a4656")
        ax.set_xlabel("x [m]", color="#9fb0c2")
        ax.set_ylabel("y [m]", color="#9fb0c2")
        ax.set_title(f"Top-down trajectories - {scene}", color="#e8eef5")
        leg = ax.legend(loc="upper right", fontsize=8, facecolor="#1b222c", edgecolor="#3a4656")
        for t in leg.get_texts():
            t.set_color("#cfdae6")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"traj_{scene}.png"), facecolor=fig.get_facecolor())
        plt.close(fig)
        print("traj", scene)


def training_curves() -> None:
    import json
    specs = [("perception", "val_iou", "val IoU"), ("planner", "val_mae_m", "val MAE [m]"),
             ("control", "val_mae_ms", "val MAE [m/s]"), ("e2e", "val_mae_ms", "val MAE [m/s]")]
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.2), dpi=150)
    fig.patch.set_facecolor("#12161c")
    for ax, (mod, key, ylab) in zip(axes.ravel(), specs):
        path = os.path.join("weights", f"{mod}_history.json")
        d = json.load(open(path))
        ep = d["epochs"]
        ax.set_facecolor("#12161c")
        ax.plot([e["epoch"] for e in ep], [e["train_loss"] for e in ep],
                color="#5ad1ff", label="train loss")
        ax2 = ax.twinx()
        ax2.plot([e["epoch"] for e in ep], [e[key] for e in ep], color="#ffb454",
                 label=ylab)
        ax.set_title(mod, color="#e8eef5")
        ax.set_xlabel("epoch", color="#9fb0c2")
        ax.set_ylabel("train loss", color="#5ad1ff")
        ax2.set_ylabel(ylab, color="#ffb454")
        ax.tick_params(colors="#9fb0c2")
        ax2.tick_params(colors="#ffb454")
        for s in list(ax.spines.values()) + list(ax2.spines.values()):
            s.set_color("#3a4656")
    fig.suptitle("Training curves (NumPy autograd, CPU only)", color="#e8eef5")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "training_curves.png"), facecolor=fig.get_facecolor())
    plt.close(fig)
    print("curves")


def bench_figure() -> None:
    import csv
    rows = list(csv.DictReader(open(os.path.join("results", "planner_bench.csv"))))
    scenes = [s for s in SCENES if any(r["scene"] == s for r in rows)]
    classical = [int(next(r["expanded"] for r in rows if r["scene"] == s and r["method"] == "classical")) for s in scenes]
    neural = [int(next(r["expanded"] for r in rows if r["scene"] == s and r["method"] == "neural" and float(r["weight"]) == 1.15)) for s in scenes]
    x = np.arange(len(scenes))
    fig, ax = plt.subplots(figsize=(8, 4.2), dpi=150)
    fig.patch.set_facecolor("#12161c")
    ax.set_facecolor("#12161c")
    ax.bar(x - 0.2, classical, 0.4, color="#5ad1ff", label="classical A*")
    ax.bar(x + 0.2, neural, 0.4, color="#ffb454", label="neural-heuristic A* (w=1.15)")
    for i, (c, n) in enumerate(zip(classical, neural)):
        ax.text(i - 0.2, c + 30, str(c), ha="center", color="#9fb0c2", fontsize=8)
        ax.text(i + 0.2, n + 30, str(n), ha="center", color="#9fb0c2", fontsize=8)
    ax.set_xticks(x, scenes)
    ax.set_ylabel("expanded nodes", color="#9fb0c2")
    ax.set_title("Search effort: neural heuristic cuts expansions on dense scenes", color="#e8eef5")
    ax.tick_params(colors="#9fb0c2")
    for s in ax.spines.values():
        s.set_color("#3a4656")
    leg = ax.legend(facecolor="#1b222c", edgecolor="#3a4656")
    for t in leg.get_texts():
        t.set_color("#cfdae6")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "planner_bench.png"), facecolor=fig.get_facecolor())
    plt.close(fig)
    print("bench")


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    stack = NeuralStack("weights")
    scene_snapshots()
    perception_figure(stack)
    planning_figure(stack)
    trajectories(stack)
    training_curves()
    bench_figure()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
