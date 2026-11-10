#!/usr/bin/env python3
"""Train PPO or SAC with Stable-Baselines3 and export a NumPy actor.

The algorithm set and network layout follow the reference project
``zhaoran2000/DRL-drones`` (``Sol/Model/PBDroneSimulator.py``):
PPO with separate 256x256 policy/value trunks and tanh activations, and SAC
with a shared 256x256 MLP, automatic entropy tuning and ``tau = 0.005``.

中文说明：使用 SB3 训练 PPO/SAC，训练结束后把 Actor 网络导出为纯 NumPy 的
``policy_weights.npz``，部署到只装了 ROS 的机器时无需 PyTorch。

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
import time
from typing import Any, Dict, Mapping

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from common import DEFAULT_CONFIG_PATH, PACKAGE_DIR, load_config  # noqa: E402
from drone_env import DronePathEnv  # noqa: E402
from policy import export_sb3_policy  # noqa: E402


def parse_args() -> argparse.Namespace:
    """Command-line interface."""
    parser = argparse.ArgumentParser(
        description="Train PPO/SAC for drl_drone_path_planning"
    )
    parser.add_argument("--algo", choices=("PPO", "SAC"), default="PPO")
    parser.add_argument("--timesteps", type=int, default=1000000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--ent-coef", type=float, default=0.003,
                        help="PPO exploration coefficient")
    parser.add_argument("--eval-freq", type=int, default=20000)
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--out",
        default=os.path.join(PACKAGE_DIR, "models"),
        help="output directory for model.zip and policy_weights.npz",
    )
    parser.add_argument("--eval-episodes", type=int, default=50)
    parser.add_argument("--device", default="cpu")
    return parser.parse_args()


def build_model(algo: str,
                env: Any,
                seed: int,
                device: str,
                ent_coef: float = 0.003) -> Any:
    """Create an SB3 model with the configured training hyperparameters."""
    from stable_baselines3 import PPO, SAC
    from torch import nn

    if algo == "PPO":
        policy_kwargs = {
            "net_arch": {"pi": [256, 256], "vf": [256, 256]},
            "activation_fn": nn.Tanh,
            "log_std_init": -0.5,  # std ≈ 0.61, less erratic early exploration
        }
        return PPO(
            "MlpPolicy",
            env,
            learning_rate=3.0e-4,
            n_steps=2048,
            batch_size=256,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=ent_coef,
            vf_coef=0.5,
            max_grad_norm=0.5,
            policy_kwargs=policy_kwargs,
            verbose=1,
            seed=seed,
            device=device,
        )

    if algo == "SAC":
        policy_kwargs = {
            "net_arch": [256, 256],
            "activation_fn": nn.ReLU,
        }
        return SAC(
            "MlpPolicy",
            env,
            learning_rate=3.0e-4,
            buffer_size=200000,
            learning_starts=1000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            train_freq=1,
            gradient_steps=1,
            ent_coef="auto",
            policy_kwargs=policy_kwargs,
            verbose=1,
            seed=seed,
            device=device,
        )

    raise ValueError("unknown algorithm: %s" % algo)


def evaluate(model: Any,
             cfg: Mapping[str, Any],
             episodes: int,
             seed: int = 0) -> Dict[str, float]:
    """Run deterministic episodes and aggregate navigation metrics."""
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
            action, _ = model.predict(observation, deterministic=True)
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


def main() -> None:
    """Train, save, export and evaluate."""
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)
    cfg = load_config(args.config)

    try:
        from stable_baselines3.common.callbacks import EvalCallback
        from stable_baselines3.common.monitor import Monitor
    except ImportError as exc:
        raise SystemExit(
            "stable-baselines3 is required for training; install with "
            "`pip install -r requirements.txt`"
        ) from exc

    env = Monitor(DronePathEnv(cfg))
    model = build_model(
        args.algo, env, args.seed, args.device, ent_coef=args.ent_coef
    )
    print("\n[train] algo=%s timesteps=%d obs_dim=%d"
          % (args.algo, args.timesteps, env.observation_space.shape[0]))

    callbacks = []
    if args.eval_freq > 0:
        eval_cfg = copy.deepcopy(cfg)
        # 训练期评估固定起点，降低指标方差；最终指标仍用带随机起点的配置。
        eval_cfg["env"]["randomize_start"] = False
        eval_env = Monitor(DronePathEnv(eval_cfg))
        callbacks.append(EvalCallback(
            eval_env,
            best_model_save_path=args.out,
            log_path=args.out,
            eval_freq=args.eval_freq,
            n_eval_episodes=20,
            deterministic=True,
            verbose=1,
        ))

    def display_path(path: str) -> str:
        """Store a portable, package-relative POSIX path in the JSON summary."""
        relative = os.path.relpath(path, PACKAGE_DIR)
        return relative.replace(os.sep, "/")

    started = time.perf_counter()
    model.learn(total_timesteps=args.timesteps, callback=callbacks)
    elapsed = time.perf_counter() - started

    best_path = os.path.join(args.out, "best_model.zip")
    used_best = os.path.isfile(best_path)
    if used_best:
        model = type(model).load(best_path, env=env)

    model_path = os.path.join(args.out, "model.zip")
    model.save(model_path)
    weights_path = os.path.join(args.out, "policy_weights.npz")
    export_info = export_sb3_policy(model, weights_path)
    export_info["path"] = display_path(weights_path)
    metrics = evaluate(model, cfg, args.eval_episodes, seed=args.seed + 1000)

    summary = {
        "algo": args.algo,
        "timesteps": args.timesteps,
        "seed": args.seed,
        "ent_coef": args.ent_coef,
        "elapsed_seconds": elapsed,
        "used_best_model": used_best,
        "model_path": display_path(model_path),
        "weights_path": display_path(weights_path),
        "metrics": metrics,
        "export": export_info,
    }
    with open(os.path.join(args.out, "train_summary.json"), "w",
              encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, ensure_ascii=False)

    print("\n[train] finished in %.1f s" % elapsed)
    print("[train] weights -> %s" % weights_path)
    for key, value in metrics.items():
        print("[eval] %-24s %.4f" % (key, value))


if __name__ == "__main__":
    main()
