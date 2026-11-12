#!/usr/bin/env python3
"""Re-evaluate the saved RL agents on the discrete muscle-activation task.

Loads ``models/rl_dqn.npz`` / ``rl_dqn_3x64.npz`` / ``rl_pg.npz`` and rewrites
``results/benchmark_rl.json`` (plus the per-model JSON metadata) with a fresh
evaluation, so the RL comparison table can be regenerated without retraining.
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from msk_arm_nn import config as C
from msk_arm_nn.datasets import DiscreteArmEnv
from msk_arm_nn.nn.rl import DQN, PolicyGradient

SCENE = "reach"


def eval_agent(agent, kind: str, episodes: int, seed: int) -> dict:
    env = DiscreteArmEnv(SCENE, seed=seed + 999, random_start=True)
    returns, steps, collisions, reached = [], [], [], []
    successes = 0
    for _ in range(int(episodes)):
        state = env.reset()
        done = False
        total = 0.0
        info = {"success": False}
        while not done:
            if kind == "dqn":
                action = agent.act(state, 0.0)
            else:
                action = agent.act_greedy(state)
            state, reward, done, info = env.step(action)
            total += reward
        successes += int(info["success"])
        returns.append(total)
        steps.append(env.steps)
        collisions.append(info.get("collisions", env.sim.collisions))
        reached.append(info.get("reached", env.sim.reached))
    return {
        "episodes": int(episodes),
        "success_rate": round(successes / float(episodes), 3),
        "mean_return": round(float(np.mean(returns)), 3),
        "mean_steps": round(float(np.mean(steps)), 1),
        "mean_collisions": round(float(np.mean(collisions)), 2),
        "mean_reached": round(float(np.mean(reached)), 2),
    }


def main() -> int:
    t0 = time.time()
    models = C.MODELS_DIR
    dqn2 = DQN.load(os.path.join(models, "rl_dqn.npz"))
    dqn3 = DQN.load(os.path.join(models, "rl_dqn_3x64.npz"))
    pg2 = PolicyGradient.load(os.path.join(models, "rl_pg.npz"))
    res2 = eval_agent(dqn2, "dqn", 30, 2026)
    res3 = eval_agent(dqn3, "dqn", 30, 2026)
    respg = eval_agent(pg2, "pg", 30, 2026)
    res2.update({"algorithm": "DQN", "hidden": [64, 64]})
    res3.update({"algorithm": "DQN", "hidden": [64, 64, 64]})
    respg.update({"algorithm": "REINFORCE", "hidden": [64, 64]})

    # keep the training metadata already stored next to the weights
    def _load_extra(name):
        path = os.path.join(models, name + ".json")
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as fh:
                return json.load(fh)
        return {}

    for name, res in (("rl_dqn", res2), ("rl_dqn_3x64", res3),
                      ("rl_pg", respg)):
        extra = _load_extra(name)
        extra.update(res)
        with open(os.path.join(models, name + ".json"), "w",
                  encoding="utf-8") as fh:
            json.dump(extra, fh, indent=2, sort_keys=True)

    env = DiscreteArmEnv(SCENE, seed=2026, random_start=False)
    summary = {
        "task": "reach the target with a muscle-driven two-link arm "
                "(discrete activation primitives, real Hill-muscle dynamics)",
        "env": {"scene": SCENE, "actions": 8,
                "action_table": DiscreteArmEnv.ACTION_TABLE.tolist(),
                "dt": 0.1, "max_steps": 80, "random_start": True,
                "state": "28-dim arm/target/depth features",
                "penalties": "collision -1.5, effort -0.05*delta, step -0.02"},
        "state_dim": int(env.state_dim),
        "eval_episodes": 30,
        "seed": 2026,
        "algorithms": {"dqn_2x64": res2, "dqn_3x64": res3, "pg_2x64": respg},
        "hidden_layer_comparison": {
            "algorithm": "DQN",
            "two_hidden_layers": {
                "success_rate": res2["success_rate"],
                "mean_return": res2["mean_return"],
                "train_seconds": _load_extra("rl_dqn").get("train_seconds"),
            },
            "three_hidden_layers": {
                "success_rate": res3["success_rate"],
                "mean_return": res3["mean_return"],
                "train_seconds": _load_extra("rl_dqn_3x64").get(
                    "train_seconds"),
            },
        },
        "seconds": round(time.time() - t0, 2),
        "regenerated": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    with open(os.path.join(C.RESULTS_DIR, "benchmark_rl.json"), "w",
              encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
    print("RL re-evaluation (30 episodes):")
    for name, metrics in summary["algorithms"].items():
        print("  %-10s success %.2f  return %.2f  steps %.1f  coll %.2f  "
              "reached %.2f" % (name, metrics["success_rate"],
                                metrics["mean_return"],
                                metrics["mean_steps"],
                                metrics["mean_collisions"],
                                metrics["mean_reached"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
