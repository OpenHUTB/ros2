#!/usr/bin/env python3
"""Offline evaluation for the DRL policy and the geometric baseline.

Metrics are reported on the same environment used for training, so a policy
that only works in the ROS node but not here (or vice versa) is immediately
visible.  ``--baseline`` runs both controllers with identical seeds.

中文说明：离线评估脚本，使用与训练完全相同的环境，对比 DRL 策略与几何基线。
指标包括成功率、碰撞率、超时率、平均回报、平均步数与平均路径长度。

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Callable, Dict, Mapping

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from baseline import baseline_action  # noqa: E402
from common import (  # noqa: E402
    DEFAULT_CONFIG_PATH,
    DEFAULT_MODEL_PATH,
    load_config,
)
from drone_env import DronePathEnv  # noqa: E402
from policy import NumpyActor  # noqa: E402

Controller = Callable[[np.ndarray], np.ndarray]


def run_episodes(controller: Controller,
                 cfg: Mapping[str, Any],
                 episodes: int,
                 seed: int = 0) -> Dict[str, float]:
    """Run ``episodes`` deterministic episodes and aggregate metrics."""
    env = DronePathEnv(cfg)
    rewards = []
    successes = 0
    collisions = 0
    timeouts = 0
    steps = []
    paths = []
    waypoints = []
    for episode in range(episodes):
        observation, _ = env.reset(seed=seed + episode)
        done = False
        total_reward = 0.0
        info: Dict[str, Any] = {}
        while not done:
            action = controller(observation)
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += float(reward)
            done = terminated or truncated
        rewards.append(total_reward)
        successes += int(bool(info.get("success", False)))
        collisions += int(bool(info.get("collision", False)))
        timeouts += int(not info.get("success", False)
                        and not info.get("collision", False))
        steps.append(float(info.get("steps", 0)))
        paths.append(float(info.get("path_length", 0.0)))
        waypoints.append(float(info.get("waypoints_reached", 0)))
    return {
        "episodes": float(episodes),
        "success_rate": successes / float(episodes),
        "collision_rate": collisions / float(episodes),
        "timeout_rate": timeouts / float(episodes),
        "mean_reward": float(np.mean(rewards)) if rewards else 0.0,
        "mean_steps": float(np.mean(steps)) if steps else 0.0,
        "mean_path_length": float(np.mean(paths)) if paths else 0.0,
        "mean_waypoints_reached": float(np.mean(waypoints)) if waypoints else 0.0,
    }


def print_metrics(name: str, metrics: Mapping[str, float]) -> None:
    """Pretty-print one metrics block."""
    print("\n[%s]" % name)
    for key, value in metrics.items():
        print("  %-24s %.4f" % (key, value))


def parse_args() -> argparse.Namespace:
    """Command-line interface."""
    parser = argparse.ArgumentParser(description="Evaluate a trained policy")
    parser.add_argument("--model", default=DEFAULT_MODEL_PATH)
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--baseline", action="store_true",
                        help="also evaluate the geometric baseline")
    parser.add_argument("--json", default="",
                        help="optional path for a JSON metrics file")
    return parser.parse_args()


def main() -> None:
    """Load a NumPy actor and evaluate it."""
    args = parse_args()
    cfg = load_config(args.config)
    if not os.path.isfile(args.model):
        raise SystemExit(
            "policy weights not found: %s\n"
            "train first: python3 train.py --algo PPO --timesteps 1000000" % args.model
        )
    actor = NumpyActor(args.model)
    print("[eval] model: %s" % actor.describe())

    results = {"drl": run_episodes(
        lambda obs: actor.predict(obs, deterministic=True),
        cfg, args.episodes, args.seed,
    )}
    if args.baseline:
        results["baseline"] = run_episodes(
            lambda obs: baseline_action(obs, cfg),
            cfg, args.episodes, args.seed,
        )

    for name, metrics in results.items():
        print_metrics(name, metrics)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(results, handle, indent=2, ensure_ascii=False)
        print("\n[eval] wrote %s" % args.json)


if __name__ == "__main__":
    main()
