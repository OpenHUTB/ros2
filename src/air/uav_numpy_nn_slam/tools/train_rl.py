#!/usr/bin/env python3
"""Compare two RL algorithms on the discrete-action aerial navigation task.

Trains DQN (2 x 64 hidden units) and REINFORCE policy gradient (2 x 64) on the
same task, plus a third run that varies the DQN depth (3 x 64) to show the
capacity comparison requested in the report.  The environment is the ``city``
airspace: buildings + border are obstacles and the no-fly zone is traversable
but penalised, so the agent has to learn to skirt it.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from uav_numpy_nn_slam import config as C
from uav_numpy_nn_slam.datasets import DiscreteAirspaceEnv
from uav_numpy_nn_slam.nn.rl import DQN, PolicyGradient

SCENE = "city"


def evaluate(agent, kind: str, episodes: int, seed: int) -> dict:
    env = DiscreteAirspaceEnv(SCENE, seed=seed + 999, random_start=True)
    returns, steps, collisions, nfz = [], [], [], []
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
        collisions.append(env.collisions)
        nfz.append(env.nfz_violations)
    return {
        "episodes": int(episodes),
        "success_rate": round(successes / float(episodes), 3),
        "mean_return": round(float(np.mean(returns)), 3),
        "mean_steps": round(float(np.mean(steps)), 1),
        "mean_collisions": round(float(np.mean(collisions)), 2),
        "mean_nfz_violations": round(float(np.mean(nfz)), 2),
    }


def quick_eval(agent, seed: int, episodes: int = 6) -> tuple:
    """Greedy evaluation on a few episodes, used to keep the best DQN."""
    env = DiscreteAirspaceEnv(SCENE, seed=seed, random_start=True)
    successes = 0
    total_return = 0.0
    for _ in range(int(episodes)):
        state = env.reset()
        done = False
        info = {"success": False}
        ret = 0.0
        while not done:
            action = agent.act(state, 0.0)
            state, reward, done, info = env.step(action)
            ret += reward
        successes += int(info["success"])
        total_return += ret
    return successes / float(episodes), total_return / float(episodes)


def train_dqn(hidden, episodes: int, args) -> dict:
    env = DiscreteAirspaceEnv(SCENE, seed=args.seed, random_start=True)
    agent = DQN(env.state_dim, env.n_actions, hidden=hidden, seed=args.seed,
                lr=args.lr, gamma=args.gamma, batch_size=32,
                target_sync=200, buffer_size=30000)
    recent = deque(maxlen=30)
    total_steps = 0
    best_score = (-1.0, -1e9)
    best_state = None
    best_epoch = 0
    t0 = time.time()
    for ep in range(1, episodes + 1):
        state = env.reset()
        done = False
        info = {"success": False}
        decay = max(1, int(0.65 * episodes))
        epsilon = max(0.05, 1.0 - 1.9 * ep / decay)
        while not done:
            action = agent.act(state, epsilon)
            nxt, reward, done, info = env.step(action)
            agent.remember(state, action, reward, nxt, done)
            if total_steps % args.train_every == 0:
                agent.train_step()
            state = nxt
            total_steps += 1
        recent.append(1.0 if info["success"] else 0.0)
        if ep % 25 == 0:
            q_succ, q_ret = quick_eval(agent, args.seed + 5000 + ep)
            if (q_succ, q_ret) > best_score:
                best_score = (q_succ, q_ret)
                best_state = [p.copy() for p in agent.net.params()]
                best_epoch = ep
            print("  DQN%s ep %3d  epsilon %.2f  recent %.2f  greedy_eval "
                  "succ %.2f ret %.2f" % (hidden, ep, epsilon,
                                          float(np.mean(recent)), q_succ,
                                          q_ret))
    if best_state is not None:
        for p, q in zip(agent.net.params(), best_state):
            p[...] = q
        agent.update_target()
    seconds = time.time() - t0
    result = evaluate(agent, "dqn", args.eval_episodes, args.seed)
    result.update({"algorithm": "DQN", "hidden": list(hidden),
                   "episodes": int(episodes), "train_steps": int(total_steps),
                   "best_epoch": int(best_epoch),
                   "train_seconds": round(seconds, 2),
                   "final_recent_success": round(float(np.mean(recent)), 3)})
    return {"agent": agent, "metrics": result}


def train_pg(hidden, episodes: int, args) -> dict:
    env = DiscreteAirspaceEnv(SCENE, seed=args.seed + 3, random_start=True)
    agent = PolicyGradient(env.state_dim, env.n_actions, hidden=hidden,
                           seed=args.seed + 5, lr=args.pg_lr,
                           gamma=args.gamma)
    recent = deque(maxlen=30)
    total_steps = 0
    t0 = time.time()
    for ep in range(1, episodes + 1):
        state = env.reset()
        states, actions, rewards = [], [], []
        done = False
        info = {"success": False}
        while not done:
            action, _ = agent.act(state)
            nxt, reward, done, info = env.step(action)
            states.append(state)
            actions.append(action)
            rewards.append(reward)
            state = nxt
            total_steps += 1
        returns = PolicyGradient.compute_returns(rewards, args.gamma)
        agent.update(np.asarray(states), actions, returns)
        recent.append(1.0 if info["success"] else 0.0)
        if ep % 100 == 0:
            print("  PG%s ep %3d  recent success %.2f  loss %.4f"
                  % (hidden, ep, float(np.mean(recent)),
                     agent.last_loss or 0.0))
    seconds = time.time() - t0
    result = evaluate(agent, "pg", args.eval_episodes, args.seed)
    result.update({"algorithm": "REINFORCE", "hidden": list(hidden),
                   "episodes": int(episodes), "train_steps": int(total_steps),
                   "train_seconds": round(seconds, 2),
                   "final_recent_success": round(float(np.mean(recent)), 3)})
    return {"agent": agent, "metrics": result}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=250,
                        help="training episodes per DQN run")
    parser.add_argument("--pg-episodes", type=int, default=250)
    parser.add_argument("--eval-episodes", type=int, default=30)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--pg-lr", type=float, default=3e-3)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--train-every", type=int, default=2)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--models-dir", default=C.MODELS_DIR)
    parser.add_argument("--results-dir", default=C.RESULTS_DIR)
    args = parser.parse_args()

    t0 = time.time()
    print("task: discrete 8-action aerial navigation in %r (point UAV, "
          "occupancy map, NFZ penalty)" % SCENE)
    print("run 1/3: DQN 2 hidden layers")
    dqn2 = train_dqn((64, 64), args.episodes, args)
    print("run 2/3: DQN 3 hidden layers")
    dqn3 = train_dqn((64, 64, 64), args.episodes, args)
    print("run 3/3: REINFORCE policy gradient, 2 hidden layers")
    pg2 = train_pg((64, 64), args.pg_episodes, args)
    elapsed = time.time() - t0

    os.makedirs(args.models_dir, exist_ok=True)
    os.makedirs(args.results_dir, exist_ok=True)
    env_meta = {"scene": SCENE, "actions": 8, "step_size": 1.0,
                "max_steps": 70, "random_start": True,
                "state": "7x7 local occupancy + goal polar + NFZ (55 dims)",
                "nfz_penalty": "reward -0.05 per step inside, count violations"}
    env = DiscreteAirspaceEnv(SCENE, seed=args.seed, random_start=False)
    summary = {
        "task": "reach goal in the city airspace (discrete action abstraction, "
                "buildings blocked, no-fly zone penalised)",
        "env": env_meta,
        "state_dim": int(env.state_dim),
        "eval_episodes": int(args.eval_episodes),
        "seed": args.seed,
        "algorithms": {
            "dqn_2x64": dqn2["metrics"],
            "dqn_3x64": dqn3["metrics"],
            "pg_2x64": pg2["metrics"],
        },
        "hidden_layer_comparison": {
            "algorithm": "DQN",
            "two_hidden_layers": {
                "success_rate": dqn2["metrics"]["success_rate"],
                "mean_return": dqn2["metrics"]["mean_return"],
                "train_seconds": dqn2["metrics"]["train_seconds"],
            },
            "three_hidden_layers": {
                "success_rate": dqn3["metrics"]["success_rate"],
                "mean_return": dqn3["metrics"]["mean_return"],
                "train_seconds": dqn3["metrics"]["train_seconds"],
            },
        },
        "seconds": round(elapsed, 2),
    }
    dqn2["agent"].save(os.path.join(args.models_dir, "rl_dqn.npz"),
                       extra=dqn2["metrics"])
    dqn3["agent"].save(os.path.join(args.models_dir, "rl_dqn_3x64.npz"),
                       extra=dqn3["metrics"])
    pg2["agent"].save(os.path.join(args.models_dir, "rl_pg.npz"),
                      extra=pg2["metrics"])
    with open(os.path.join(args.results_dir, "benchmark_rl.json"), "w",
              encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)

    print("\nRL comparison (eval over %d episodes):" % args.eval_episodes)
    print("  %-12s %8s %10s %10s %8s" % ("algorithm", "success", "return",
                                         "steps", "nfz"))
    for name, metrics in summary["algorithms"].items():
        print("  %-12s %8.2f %10.2f %10.1f %8.2f"
              % (name, metrics["success_rate"], metrics["mean_return"],
                 metrics["mean_steps"], metrics["mean_nfz_violations"]))
    print("  total %.1f s" % elapsed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
