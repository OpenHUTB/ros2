#!/usr/bin/env python3
"""
Demonstration warm-start (behaviour cloning) for the PPO navigator.

Why this exists
---------------
Training PPO on this task *from a random initialisation* is a well known
exploration trap, and we hit it head-on:

  * a do-nothing hover is a strong local optimum (its return is better than
    every naive forward policy, all of which crash into the slalom gates);
  * SB3's default ``log_std_init = 0`` makes the initial action distribution
    ~N(0, 1) across the whole [-1, 1] box, so early rollouts are violent and
    the vehicle is destroyed within roughly a second - the agent never gets to
    observe a successful transit;
  * the reward is dominated by terminal terms (order 40) versus per-step
    shaping (order 1e-2), which swamps the advantage estimator.

Warm-starting the actor from a simple pure-pursuit controller removes the
exploration problem entirely.  This is standard practice for flight control
("learn to fly by imitating, then improve with RL") and is reported as such:
the *contribution* being evaluated is the safe-RL mechanism, not the raw
ability of PPO to discover flight from noise.

The script:

  1. rolls out the pure-pursuit expert in the exact training environment
     (same obstacles, gusts, dynamic obstacles and CBF setting),
  2. fits the PPO actor's action mean to the expert actions by MSE,
  3. saves the resulting weights so ``train.py --init-from`` can fine-tune.

Everything is written to the standard evidence directories.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch as th

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from run_logging import RunLogger, ensure_dirs, write_run_manifest  # noqa: E402
from safe_nav_env import EnvConfig, SafeNavEnv                          # noqa: E402


# --------------------------------------------------------------------------
# expert
# --------------------------------------------------------------------------


def pure_pursuit_action(env: SafeNavEnv, gain: float = 0.9, yaw_gain: float = 0.0) -> np.ndarray:
    """Point straight at the active gate, normalised to the action box.

    Deliberately trivial: it ignores obstacles entirely and relies on the CBF
    filter (when enabled) plus the dynamics to survive.  That is the point -
    the demonstrations need only be *good enough to be worth imitating*, and
    the student subsequently improves on them with PPO.
    """
    pos = env.drone_state()["position"]
    gate = env.gates[min(env._gate_idx, env.cfg.n_gates - 1)]
    delta = gate - pos
    n = float(np.linalg.norm(delta))
    u = delta / n if n > 1e-6 else np.zeros(3)
    return np.array([u[0] * gain, u[1] * gain, u[2] * gain, yaw_gain], dtype=np.float32)


def collect_demonstrations(env: SafeNavEnv, n_episodes: int, seed0: int = 0,
                           vel_gain: float = 0.9, noise: float = 0.0):
    """Return ``(observations, actions)`` from expert rollouts."""
    rng = np.random.default_rng(seed0)
    obs_list, act_list = [], []
    stats = {"success": 0, "gates": [], "len": []}
    for ep in range(n_episodes):
        obs, info = env.reset(seed=seed0 + ep)
        done = False
        n = 0
        while not done:
            a = pure_pursuit_action(env, gain=vel_gain)
            if noise > 0.0:
                a = np.clip(a + rng.normal(0.0, noise, size=4), -1.0, 1.0).astype(np.float32)
            obs_list.append(np.asarray(obs, dtype=np.float32))
            act_list.append(np.asarray(a, dtype=np.float32))
            obs, _r, term, trunc, info = env.step(a)
            done = term or trunc
            n += 1
        stats["success"] += int(info.get("is_success", False))
        stats["gates"].append(info.get("gate_index", 0))
        stats["len"].append(n)
    return (np.asarray(obs_list, dtype=np.float32),
            np.asarray(act_list, dtype=np.float32), stats)


# --------------------------------------------------------------------------
# behaviour cloning
# --------------------------------------------------------------------------


def bc_fit(model, obs: np.ndarray, act: np.ndarray, epochs: int, batch_size: int,
           lr: float, device: str = "cpu", log_every: int = 50):
    """Fit the PPO actor's mean action to the expert actions."""
    policy = model.policy
    policy.set_training_mode(True)

    # Freeze everything except the shared trunk + the action head.
    for p in policy.parameters():
        p.requires_grad_(False)
    for p in policy.mlp_extractor.parameters():
        p.requires_grad_(True)
    for p in policy.features_extractor.parameters():
        p.requires_grad_(True)
    for p in policy.action_net.parameters():
        p.requires_grad_(True)

    params = [p for p in policy.parameters() if p.requires_grad]
    opt = th.optim.Adam(params, lr=lr)

    n = obs.shape[0]
    idx_all = np.arange(n)
    history = []
    for epoch in range(epochs):
        perm = np.random.permutation(idx_all)
        losses = []
        for start in range(0, n, batch_size):
            mb = perm[start:start + batch_size]
            if mb.size < 2:
                continue
            obs_t = th.as_tensor(obs[mb], device=device)
            act_t = th.as_tensor(act[mb], device=device)

            latent_pi, _ = policy.mlp_extractor(policy.extract_features(obs_t))
            pred = policy.action_net(latent_pi)
            loss = th.nn.functional.mse_loss(pred, act_t)

            opt.zero_grad()
            loss.backward()
            th.nn.utils.clip_grad_norm_(params, 1.0)
            opt.step()
            losses.append(loss.item())

        mean_loss = float(np.mean(losses)) if losses else float("nan")
        history.append(mean_loss)
        if epoch % log_every == 0 or epoch == epochs - 1:
            print(f"    [bc] epoch {epoch:4d}/{epochs}  mse={mean_loss:.6f}")

    policy.set_training_mode(False)
    return history


def evaluate_policy(model, env: SafeNavEnv, n_episodes: int, seed0: int = 5000):
    success, gates, lens, margins = 0, [], [], []
    for ep in range(n_episodes):
        obs, info = env.reset(seed=seed0 + ep)
        done = False
        n = 0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _r, term, trunc, info = env.step(action)
            done = term or trunc
            n += 1
        success += int(info.get("is_success", False))
        gates.append(info.get("gate_index", 0))
        lens.append(n)
        margins.append(info.get("min_margin", np.nan))
    return {
        "success_rate": success / max(n_episodes, 1),
        "mean_gates": float(np.mean(gates)),
        "mean_length": float(np.mean(lens)),
        "mean_min_margin": float(np.nanmean(margins)),
    }


# --------------------------------------------------------------------------


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--episodes", type=int, default=220, help="expert episodes to collect")
    p.add_argument("--epochs", type=int, default=300)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--noise", type=float, default=0.06,
                   help="action noise added to the expert for state-space coverage")
    p.add_argument("--vel-gain", type=float, default=0.9)
    p.add_argument("--use-cbf", action="store_true")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--max-steps", type=int, default=900)
    p.add_argument("--n-gates", type=int, default=3)
    p.add_argument("--n-static", type=int, default=7)
    p.add_argument("--n-dynamic", type=int, default=4)
    p.add_argument("--log-std-init", type=float, default=-1.0)
    p.add_argument("--out-name", type=str, default=None)
    p.add_argument("--device", default="cpu")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    paths = ensure_dirs()
    name = args.out_name or f"bc_pretrain{'_cbf' if args.use_cbf else ''}"

    with RunLogger(paths["logs"], name=name) as rl:
        rl.section(f"SafeDRL-DroneNav-ROS :: BEHAVIOUR-CLONING WARM-START  [{name}]")
        rl.log(f"expert            = pure pursuit (gain {args.vel_gain}, noise {args.noise})")
        rl.log(f"expert use_cbf    = {args.use_cbf}")
        rl.log(f"expert episodes   = {args.episodes}")

        overrides = dict(
            gui=False, use_cbf=args.use_cbf, max_steps=args.max_steps,
            n_gates=args.n_gates, n_static=args.n_static, n_dynamic=args.n_dynamic,
        )
        env = SafeNavEnv(EnvConfig(**overrides))

        t0 = time.time()
        obs, act, stats = collect_demonstrations(
            env, args.episodes, seed0=args.seed, vel_gain=args.vel_gain, noise=args.noise
        )
        rl.section("Expert rollouts")
        rl.log(f"transitions       = {obs.shape[0]}  obs_dim={obs.shape[1]}")
        rl.log(f"expert success    = {stats['success']}/{args.episodes} "
               f"({stats['success'] / args.episodes:.1%})")
        rl.log(f"expert mean gates = {np.mean(stats['gates']):.2f}")
        rl.log(f"expert mean length= {np.mean(stats['len']):.1f} steps")
        rl.log(f"collection time   = {time.time() - t0:.1f} s")

        # ---- build the model whose actor we are fitting -------------------
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv

        venv = DummyVecEnv([lambda: env])
        model = PPO(
            "MlpPolicy", venv, n_steps=1024, batch_size=256, verbose=0,
            seed=args.seed, device=args.device,
            policy_kwargs=dict(
                net_arch=dict(pi=[128, 128], vf=[128, 128]),
                log_std_init=args.log_std_init,
            ),
        )

        rl.section("Behaviour cloning")
        history = bc_fit(model, obs, act, args.epochs, args.batch_size, args.lr,
                         device=args.device)
        rl.log(f"initial mse = {history[0]:.6f}")
        rl.log(f"final   mse = {history[-1]:.6f}")

        rl.section("Cloned policy evaluation")
        metrics = evaluate_policy(model, env, n_episodes=16, seed0=5000)
        for k, v in metrics.items():
            rl.log(f"{k:<20s} = {v:.4f}")

        ckpt = paths["checkpoints"] / name
        model.save(str(ckpt))
        rl.log(f"saved warm-start  : {ckpt}.zip")

        np.savetxt(paths["logs"] / f"{name}_bc_loss.csv", np.asarray(history),
                   delimiter=",", header="mse", comments="")

        write_run_manifest(paths, {
            "run_name": name, "kind": "bc_pretrain",
            "expert_episodes": args.episodes, "transitions": int(obs.shape[0]),
            "expert_success": stats["success"] / args.episodes,
            "bc_initial_mse": history[0], "bc_final_mse": history[-1],
            "cloned_success_rate": metrics["success_rate"],
            "use_cbf": args.use_cbf,
        })
        print(f"\nBC_MODEL={ckpt}.zip")
        print(f"BC_SUCCESS={metrics['success_rate']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
