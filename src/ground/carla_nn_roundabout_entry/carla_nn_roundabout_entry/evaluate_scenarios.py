"""Evaluate paired deterministic roundabout episodes across random seeds."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections.abc import Iterable
from functools import lru_cache

from .dataset import make_dataset, rule_label, split_dataset
from .end_to_end import ImageToControl
from .simulator import RoundaboutEnv


CONTROLLERS = ("rule", "image-model")


def phase_for_seed(seed: int) -> float:
    """Map an integer seed to a stable NPC phase in [0, 2π)."""
    mixed = (seed * 2654435761) % (2 ** 32)
    return mixed / (2 ** 32) * 2.0 * math.pi


@lru_cache(maxsize=1)
def _trained_image_model() -> ImageToControl:
    images, controls = make_dataset(n=800, seed=7)
    train_images, train_controls, _, _ = split_dataset(images, controls, seed=7)
    model = ImageToControl(seed=7)
    model.train(train_images, train_controls, epochs=300)
    return model


def run_episode(seed: int, steps: int, controller: str) -> dict[str, float | int | bool]:
    if controller not in CONTROLLERS:
        raise ValueError("controller must be 'rule' or 'image-model'")
    if steps <= 0:
        raise ValueError("steps must be positive")

    phase = phase_for_seed(seed)
    env = RoundaboutEnv()
    observation = env.reset(npc_phase=phase)
    model = _trained_image_model() if controller == "image-model" else None
    speeds = []
    min_center_distance = float("inf")
    steps_run = 0

    for _ in range(steps):
        oncoming_x, oncoming_y = env._npc_state()
        min_center_distance = min(
            min_center_distance, math.hypot(oncoming_x, oncoming_y)
        )
        if model is None:
            action = rule_label(env.lateral, oncoming_x, oncoming_y)
        else:
            action = model.predict(observation[None, ...])[0]
        observation, speed, done, _ = env.step(action)
        speeds.append(float(speed))
        steps_run += 1
        if done:
            break

    return {
        "seed": int(seed),
        "npc_phase_rad": round(phase, 6),
        "controller": controller,
        "steps": steps_run,
        "elapsed_s": round(steps_run * env.dt, 3),
        "collision": bool(env.crashed),
        "finished": bool(env.finished),
        "average_speed_mps": round(statistics.mean(speeds), 3),
        "minimum_npc_center_distance_m": round(min_center_distance, 3),
    }


def evaluate_scenarios(
    seeds: Iterable[int],
    steps: int = 200,
    controller: str = "both",
) -> dict[str, object]:
    seed_list = list(seeds)
    if not seed_list:
        raise ValueError("at least one seed is required")
    if steps <= 0:
        raise ValueError("steps must be positive")
    if controller not in ("both", *CONTROLLERS):
        raise ValueError("controller must be 'both', 'rule', or 'image-model'")

    selected = CONTROLLERS if controller == "both" else (controller,)
    results = [
        run_episode(seed, steps, name)
        for name in selected
        for seed in seed_list
    ]
    summaries = {}
    for name in selected:
        rows = [row for row in results if row["controller"] == name]
        summaries[name] = {
            "scenario_count": len(rows),
            "collision_rate": round(statistics.mean(
                bool(row["collision"]) for row in rows
            ), 3),
            "finish_rate": round(statistics.mean(
                bool(row["finished"]) for row in rows
            ), 3),
            "mean_steps": round(statistics.mean(row["steps"] for row in rows), 3),
            "mean_speed_mps": round(statistics.mean(
                row["average_speed_mps"] for row in rows
            ), 3),
            "worst_minimum_npc_center_distance_m": min(
                row["minimum_npc_center_distance_m"] for row in rows
            ),
        }
    return {
        "seeds": seed_list,
        "step_limit": steps,
        "pairing": "同一随机种子下，各控制器使用相同的环道车流相位",
        "summary": summaries,
        "scenarios": results,
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="按固定随机种子对比环岛控制器")
    parser.add_argument("--seeds", nargs="+", type=int, default=[3, 7, 11, 19, 23])
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--controller", choices=("both", *CONTROLLERS), default="both")
    args = parser.parse_args(argv)
    report = evaluate_scenarios(args.seeds, args.steps, args.controller)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
