#!/usr/bin/env python3
"""Unified entry point for the neural collision-avoidance drone project.

Every module of the assignment is launched through this file (or through the
matching ROS launch file, which calls it)::

    python main.py --module sim        --scene dense
    python main.py --module perception --scene dense  --weights weights
    python main.py --module planning   --scene corridor --weights weights
    python main.py --module control    --scene sparse --weights weights
    python main.py --module e2e        --scene dynamic --weights weights --gif
    python main.py --module full       --scene corridor --weights weights --gif
    python main.py --module demo       --weights weights

Outputs are written to ``runs/<timestamp>/`` (figures, GIFs, metrics.json).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ca_drone.pipeline import (MAX_RANGE, NeuralStack, run_episode, save_gif)
from ca_drone.render.scene import compose_frame, depth_image
from ca_drone.sim.world import make_world

SCENES = ["sparse", "dense", "dynamic", "corridor", "unseen"]
ROOT = os.path.dirname(os.path.abspath(__file__))


def _outdir(tag: str) -> str:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(ROOT, "runs", f"{stamp}_{tag}")
    os.makedirs(path, exist_ok=True)
    return path


def _save(img, path: str):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    img.save(path)
    print(f"saved {path} ({os.path.getsize(path) / 1024:.1f} KB)")


def module_sim(args, out: str) -> dict:
    w = make_world(args.scene)
    w.t = args.t
    pos = np.array([args.x, args.y, w.flight_z])
    img = compose_frame(w, pos, np.zeros(3), args.yaw, None,
                        {"mode": "SIM", "progress": 0.0})
    _save(img, os.path.join(out, f"frame_{args.scene}.png"))
    return {"scene": args.scene, "pos": pos.tolist(), "clearance": w.clearance(pos)}


def module_perception(args, out: str) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from ca_drone.models.perception import predict_mask
    from ca_drone.render.scene import colorize_depth

    stack = NeuralStack(args.weights)
    w = make_world(args.scene)
    w.t = args.t
    rng = np.random.default_rng(args.seed)
    from ca_drone.data.collect import _sample_pose
    p = _sample_pose(w, rng) or w.start
    depth, ids = depth_image(w, p, args.yaw, 64, 48, 90.0, MAX_RANGE)
    prob = predict_mask(stack.perception, depth, MAX_RANGE)
    gt = (ids >= 0).astype(np.float32)
    pred = (prob > 0.5).astype(np.float32)
    inter = float((pred * gt).sum())
    iou = inter / max(float(pred.sum() + gt.sum() - inter), 1.0)
    dice = 2 * inter / max(float(pred.sum() + gt.sum()), 1.0)

    fig, ax = plt.subplots(1, 3, figsize=(12, 3.2), dpi=160)
    ax[0].imshow(np.array(colorize_depth(depth, MAX_RANGE)))
    ax[0].set_title("RGB-D input (colourised)", fontsize=9)
    ax[1].imshow(gt, cmap="gray", vmin=0, vmax=1)
    ax[1].set_title("ground-truth obstacle mask", fontsize=9)
    ax[2].imshow(prob, cmap="magma", vmin=0, vmax=1)
    ax[2].set_title(f"CNN probability (IoU={iou:.3f})", fontsize=9)
    for a in ax:
        a.set_xticks([])
        a.set_yticks([])
    fig.tight_layout()
    path = os.path.join(out, f"perception_{args.scene}.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"saved {path}")
    return {"scene": args.scene, "iou": iou, "dice": dice,
            "positive_px": int(gt.sum()), "infer_ms": stack.load_ms}


def module_planning(args, out: str) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from ca_drone.mapping import GridMap
    from ca_drone.pipeline import _plan
    from ca_drone.planning.astar import astar
    from ca_drone.models.planner import predict_field

    stack = NeuralStack(args.weights)
    w = make_world(args.scene)
    gm = GridMap.from_world(w, 0.5, inflate=0.45)
    occ = gm.plan_grid(margin=1)
    s, g = gm.to_cell(w.start), gm.to_cell(w.goal)
    t = time.time()
    classical = astar(occ, s, g, 0.5, weight=1.0)
    t_classic = (time.time() - t) * 1e3
    t = time.time()
    field = predict_field(stack.planner, occ, g, 0.5)
    neural = astar(occ, s, g, 0.5, heuristic=field, weight=1.15)
    t_neural = (time.time() - t) * 1e3
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2), dpi=160)
    ax[0].imshow(occ.T, origin="lower", cmap="Greys", extent=[0, 52, -22, 22])
    if classical["success"]:
        c = np.array(classical["path_cells"], dtype=float)
        ax[0].plot((c[:, 0] + 0.5) * 0.5, (c[:, 1] + 0.5) * 0.5, "-", lw=1.6,
                   color="#1f77b4", label=f"classical A* ({t_classic:.1f} ms)")
    ax[0].set_title("classical A*", fontsize=9)
    ax[0].legend(loc="upper left", fontsize=7)
    im = ax[1].imshow(field.T, origin="lower", cmap="viridis", extent=[0, 52, -22, 22],
                      vmax=np.percentile(field[np.isfinite(field)], 95) if np.any(np.isfinite(field)) else 1)
    if neural["success"]:
        c = np.array(neural["path_cells"], dtype=float)
        ax[1].plot((c[:, 0] + 0.5) * 0.5, (c[:, 1] + 0.5) * 0.5, "-", lw=1.6,
                   color="#ff7f0e", label=f"neural A* ({t_neural:.1f} ms)")
    ax[1].set_title("learned cost-to-go heuristic", fontsize=9)
    ax[1].legend(loc="upper left", fontsize=7)
    fig.colorbar(im, ax=ax[1], fraction=0.035, label="cost-to-go [m]")
    for a in ax:
        a.set_xlabel("x [m]")
        a.set_ylabel("y [m]")
    fig.tight_layout()
    path = os.path.join(out, f"planning_{args.scene}.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"saved {path}")
    return {"scene": args.scene,
            "classical": {"success": classical["success"], "expanded": classical["expanded"],
                          "ms": t_classic},
            "neural": {"success": neural["success"], "expanded": neural["expanded"],
                       "ms": t_neural}}


def _run(args, out: str, mode: str) -> dict:
    stack = NeuralStack(args.weights)
    t = time.time()
    r = run_episode(args.scene, mode, stack, max_steps=args.max_steps,
                    render_every=args.render_every, seed=args.seed,
                    verbose=True)
    info = r.as_dict()
    info["wall_s"] = round(time.time() - t, 2)
    if args.gif:
        gif = os.path.join(out, f"{mode}_{args.scene}.gif")
        save_gif(r.frames, gif, duration_ms=args.gif_ms)
        info["gif"] = gif
        print(f"saved {gif} ({os.path.getsize(gif) / 1024 / 1024:.2f} MB, "
              f"{len(r.frames)} frames)")
    elif r.frames:
        _save(r.frames[-1], os.path.join(out, f"{mode}_{args.scene}.png"))
    print(json.dumps(info, indent=2))
    return info


def module_demo(args, out: str) -> dict:
    stack = NeuralStack(args.weights)
    results = {}
    for scene in SCENES:
        t = time.time()
        r = run_episode(scene, "full", stack, max_steps=args.max_steps,
                        render_every=args.render_every, seed=args.seed, verbose=True)
        gif = os.path.join(out, f"full_{scene}.gif")
        save_gif(r.frames, gif, duration_ms=args.gif_ms)
        info = r.as_dict()
        info["gif"] = gif
        info["wall_s"] = round(time.time() - t, 2)
        results[scene] = info
        print(f"[demo] {scene}: success={info['success']} "
              f"gif={os.path.getsize(gif) / 1024 / 1024:.2f} MB")
    return results


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--module", default="full",
                    choices=["sim", "perception", "planning", "control", "e2e", "full", "demo"])
    ap.add_argument("--scene", default="sparse", choices=SCENES)
    ap.add_argument("--weights", default=os.path.join(ROOT, "weights"))
    ap.add_argument("--max-steps", type=int, default=650)
    ap.add_argument("--render-every", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--t", type=float, default=0.0)
    ap.add_argument("--x", type=float, default=6.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--gif", action="store_true")
    ap.add_argument("--gif-ms", type=int, default=90)
    args = ap.parse_args()

    out = _outdir(args.module)
    if args.module == "sim":
        res = module_sim(args, out)
    elif args.module == "perception":
        res = module_perception(args, out)
    elif args.module == "planning":
        res = module_planning(args, out)
    elif args.module in ("control", "full", "e2e"):
        res = _run(args, out, args.module)
    else:
        res = module_demo(args, out)
    with open(os.path.join(out, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    print(f"\noutputs -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
