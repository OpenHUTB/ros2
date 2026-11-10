#!/usr/bin/env python3
"""Plot a top-down trajectory comparison between DRL and the baseline.

This is an offline development utility (needs matplotlib); it is intentionally
not installed into the catkin ``bin`` directory, so the ROS deployment does not
pull in matplotlib.  Run it from the package root:

    python3 scripts/plot_results.py --model models/policy_weights.npz

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Any, Mapping

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from baseline import baseline_action  # noqa: E402
from common import (  # noqa: E402
    DEFAULT_CONFIG_PATH,
    DEFAULT_MODEL_PATH,
    PACKAGE_DIR,
    load_config,
)
from drone_env import DronePathEnv  # noqa: E402
from policy import NumpyActor  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(PACKAGE_DIR, "..", "..", ".."))
DEFAULT_OUT = os.path.join(
    REPO_ROOT, "docs", "img", "drl_drone_path_planning", "path_comparison.png"
)


def run_episode(controller: Any,
                cfg: Mapping[str, Any],
                seed: int) -> tuple:
    """Run one episode and return trajectory plus summary info."""
    env = DronePathEnv(cfg)
    observation, _ = env.reset(seed=seed)
    done = False
    info = {}
    while not done:
        action = controller(observation)
        observation, _, terminated, truncated, info = env.step(action)
        done = terminated or truncated
    return env.trajectory, info


def main() -> None:
    """Generate the comparison figure."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except ImportError as exc:
        raise SystemExit("matplotlib is required for plotting") from exc

    parser = argparse.ArgumentParser(description="Plot flight trajectories")
    parser.add_argument("--model", default=DEFAULT_MODEL_PATH)
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default=DEFAULT_OUT)
    args = parser.parse_args()

    cfg = load_config(args.config)
    actor = NumpyActor(args.model)
    drl_path, drl_info = run_episode(
        lambda obs: actor.predict(obs, deterministic=True), cfg, args.seed
    )
    base_path, base_info = run_episode(
        lambda obs: baseline_action(obs, cfg), cfg, args.seed
    )

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    figure, axis = plt.subplots(figsize=(8.0, 6.5))
    for ox, oy, radius, _height in cfg["env"]["obstacles"]:
        axis.add_patch(Circle((ox, oy), radius, color="#555555", alpha=0.65))

    waypoints = np.asarray(cfg["goal"]["waypoints"], dtype=np.float64)
    axis.plot(waypoints[:, 0], waypoints[:, 1], "--", color="#888888",
              linewidth=1.2, label="waypoint route")
    axis.scatter(waypoints[:, 0], waypoints[:, 1], marker="*", s=160,
                 color="#f2c200", edgecolor="#333333", zorder=5,
                 label="waypoints")
    axis.scatter([cfg["env"]["start"][0]], [cfg["env"]["start"][1]],
                 marker="o", s=80, color="#1f77b4", edgecolor="#ffffff",
                 zorder=6, label="start")

    axis.plot(base_path[:, 0], base_path[:, 1], color="#ff7f0e",
              linewidth=2.0, label="baseline (pursuit + potential field)")
    axis.plot(drl_path[:, 0], drl_path[:, 1], color="#2ca02c",
              linewidth=2.0, label="DRL policy (PPO)")

    title = "success: DRL=%s baseline=%s | steps: DRL=%s baseline=%s" % (
        bool(drl_info.get("success")), bool(base_info.get("success")),
        int(drl_info.get("steps", 0)), int(base_info.get("steps", 0)),
    )
    axis.set_title(title, fontsize=10)
    axis.set_xlabel("x / m")
    axis.set_ylabel("y / m")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, linestyle=":", alpha=0.5)
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(args.out, dpi=160)
    print("[plot] wrote %s" % args.out)


if __name__ == "__main__":
    main()
