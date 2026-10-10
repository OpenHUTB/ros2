#!/usr/bin/env python3
"""
Training entry point for SafeDRL-DroneNav-ROS.

Produces, in one run:

  * ``logs/tb_logs/<run>/``        TensorBoard event files (reward, loss,
                                    entropy, ep_len_mean, ep_rew_mean,
                                    constraint/lambda, cost, eval/*)
  * ``logs/train_execution.log``   full terminal transcript, appended
  * ``logs/<run>_metrics.json``    structured evaluation metrics
  * ``checkpoints/<run>*.zip``     model weights (best + final)
  * ``docs/figures/*.png``         convergence dashboard and evaluation plots

Examples
--------
Single environment, quick smoke test::

    python scripts/train.py --timesteps 20000 --n-envs 1 --run-name smoke

The full ablation reported in the README::

    python scripts/train.py --policy ppo        --run-name baseline_ppo      --timesteps 400000
    python scripts/train.py --policy ppo_lag    --run-name ppo_lagrangian    --timesteps 400000
    python scripts/train.py --policy ppo_lag --use-cbf --run-name ppo_lag_cbf --timesteps 400000
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
import time
import traceback
import zipfile
from datetime import datetime
from pathlib import Path

import numpy as np
import torch as th
from stable_baselines3.common.callbacks import BaseCallback

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

# Headless by default: never require a display in the VM.
os.environ.setdefault("MPLBACKEND", "Agg")

from safe_nav_env import EnvConfig, SafeNavEnv                       # noqa: E402
from run_logging import (RunLogger, MetricsRecorder, ensure_dirs,     # noqa: E402
                         write_run_manifest)


# ==========================================================================
# environment factory (module level so SubprocVecEnv can pickle it)
# ==========================================================================


def make_env(seed: int, overrides: dict, rank: int = 0):
    """Return a thunk building one monitored environment."""
    def _thunk():
        from stable_baselines3.common.monitor import Monitor
        cfg = EnvConfig(**overrides)
        cfg.seed = int(seed) + 1000 * rank
        env = SafeNavEnv(cfg)
        env = Monitor(env)
        return env
    return _thunk


def build_vec_env(n_envs: int, seed: int, overrides: dict, force_dummy: bool = False,
                  normalize_reward: bool = False):
    """Create a vectorised environment, preferring true parallelism.

    ``normalize_reward`` wraps the vectorised env in ``VecNormalize``.  This is
    not cosmetic: the return is a sum of per-step costs of order 0.01 together
    with terminal terms of order 40, and feeding that unnormalised scale to PPO
    makes the advantage estimates useless - the policy collapses to a constant
    near-zero output and never discovers forward flight while hovering
    remains the least-bad option.  Only rewards are normalised; observations
    are already hand-scaled in the environment.
    """
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

    if n_envs <= 1 or force_dummy:
        venv, kind = DummyVecEnv([make_env(seed, overrides, 0)]), "DummyVecEnv"
    else:
        try:
            venv = SubprocVecEnv([make_env(seed, overrides, i) for i in range(n_envs)])
            kind = "SubprocVecEnv"
        except Exception as exc:  # noqa: BLE001
            print(f"  [warn] SubprocVecEnv unavailable ({exc}); falling back to DummyVecEnv")
            venv = DummyVecEnv([make_env(seed, overrides, i) for i in range(n_envs)])
            kind = "DummyVecEnv"

    if normalize_reward:
        venv = VecNormalize(venv, norm_obs=False, norm_reward=True, clip_reward=10.0)
        kind += " + VecNormalize(norm_reward)"
    return venv, kind


# ==========================================================================
# warm start
# ==========================================================================


def _load_policy_weights(model, ckpt: Path, verbose: bool = True) -> int:
    """Copy the actor/feature weights of a checkpoint into ``model``.

    Only the *policy* tensors are transferred - the checkpoint may come from a
    different algorithm class (a plain ``PPO`` warm start feeding a
    ``LagrangianPPO`` fine-tune, which is exactly our setup), so loading the
    whole object with ``set_parameters`` would fail on the extra cost critic.
    Tensors whose shape differs (e.g. a differently sized value head) are
    skipped with a warning rather than raising, so a warm start never blocks a
    run.
    """
    if not ckpt.exists():
        if ckpt.with_suffix(".zip").exists():
            ckpt = ckpt.with_suffix(".zip")
        else:
            raise FileNotFoundError(f"warm-start checkpoint not found: {ckpt}")

    # An SB3 checkpoint is a *zip archive* holding a pickled `data` blob and a
    # `policy.pth` torch state dict - handing the .zip straight to torch.load
    # fails with "file in archive is not in a subdirectory: data".
    if ckpt.suffix == ".zip":
        with zipfile.ZipFile(ckpt) as zf:
            names = zf.namelist()
            inner = next((n for n in ("policy.pth", "policy.pt") if n in names),
                         next((n for n in names if n.endswith((".pth", ".pt"))), None))
            if inner is None:
                raise ValueError(f"no policy weights inside {ckpt}: {names}")
            with zf.open(inner) as fh:
                src_state = th.load(io.BytesIO(fh.read()), map_location="cpu",
                                    weights_only=False)
    else:
        src_state = th.load(ckpt, map_location="cpu", weights_only=False)
        src_state = src_state.get("policy", src_state) if isinstance(src_state, dict) else src_state

    dst_state = model.policy.state_dict()

    loaded, skipped = 0, []
    with th.no_grad():
        for key, tensor in src_state.items():
            if key not in dst_state:
                skipped.append(key)
                continue
            if dst_state[key].shape != tensor.shape:
                skipped.append(f"{key} (shape {tuple(tensor.shape)} != "
                               f"{tuple(dst_state[key].shape)})")
                continue
            dst_state[key].copy_(tensor)
            loaded += 1
    if verbose and skipped:
        print(f"  [warm-start] skipped {len(skipped)} tensor(s): {skipped[:4]}"
              f"{' ...' if len(skipped) > 4 else ''}")
    return loaded


# ==========================================================================
# callbacks
# ==========================================================================


class TrainingLogCallback(BaseCallback):
    """Print the per-rollout training trace that the evidence log must contain.

    Emits exactly the quantities a reviewer looks for - current step, mean
    episode reward and length, policy/value losses, entropy - plus the
    Lagrangian multiplier and mean constraint cost when the run is
    constrained.

    The numbers are pulled from ``ep_info_buffer`` and (for LagrangianPPO)
    ``model.last_metrics`` rather than from ``logger.name_to_value``: SB3's
    ``Logger.dump()`` clears that dict at the end of every logging interval,
    so reading it inside a rollout yields an empty mapping.
    """

    def __init__(self, log_every: int = 4096, verbose: int = 0):
        super().__init__(verbose)
        self.log_every = log_every
        # Start one full interval in: episode statistics only exist after the
        # first rollout has actually produced a finished episode.
        self._next = log_every
        self._t0 = time.time()
        self._last_steps = 0
        self._last_t = self._t0

    @staticmethod
    def _safe_mean(values) -> float:
        return float(np.mean(values)) if len(values) > 0 else float("nan")

    def _on_step(self) -> bool:
        if self.num_timesteps < self._next:
            return True
        ep_buf = getattr(self.model, "ep_info_buffer", None)
        if not ep_buf:
            return True
        self._next = self.num_timesteps + self.log_every

        ep_rew = self._safe_mean([e["r"] for e in ep_buf])
        ep_len = self._safe_mean([e["l"] for e in ep_buf])

        # Loss columns.  LagrangianPPO stashes them in `last_metrics`; a plain
        # SB3 PPO has no such attribute, so fall back to whatever the logger
        # still holds and finally to NaN.  (Requiring `last_metrics` outright
        # silently suppressed every training line for the unconstrained
        # baseline - the run produced no per-rollout trace at all.)
        m = getattr(self.model, "last_metrics", None)
        if not m:
            m = getattr(self.model.logger, "name_to_value", {}) or {}
        g = lambda k, d=float("nan"): m.get(k, d)  # noqa: E731

        lam = float(getattr(getattr(self.model, "lagrange", None), "lam", float("nan")))
        cost = float(getattr(self.model, "last_episode_cost", float("nan")))

        now = time.time()
        sps = (self.num_timesteps - self._last_steps) / max(now - self._last_t, 1e-9)
        self._last_t, self._last_steps = now, self.num_timesteps

        print(
            f"[train] step={self.num_timesteps:>8d}  "
            f"ep_rew_mean={ep_rew:>9.3f}  "
            f"ep_len_mean={ep_len:>7.1f}  "
            f"pg_loss={g('policy_gradient_loss'):>9.4f}  "
            f"value_loss={g('value_loss'):>9.3f}  "
            f"entropy={g('entropy_loss'):>8.4f}  "
            f"lambda={lam:>7.3f}  "
            f"cost={cost:>7.3f}  "
            f"sps={sps:>6.0f}  elapsed={now - self._t0:>6.1f}s"
        )
        return True


class SafetyEvalCallback(BaseCallback):
    """Roll out the current policy on a dedicated environment and record
    success rate, collision rate, arrival time and the minimum safety margin.
    Keeps the best-by-success checkpoint."""

    def __init__(self, eval_env: SafeNavEnv, eval_freq: int, n_episodes: int,
                 out_dir: Path, run_name: str, checkpoint_dir: Path | None = None,
                 verbose: int = 0):
        super().__init__(verbose)
        self.eval_env = eval_env
        self.eval_freq = eval_freq
        self.n_episodes = n_episodes
        self.out_dir = Path(out_dir)
        self.checkpoint_dir = Path(checkpoint_dir or (self.out_dir.parent / "checkpoints"))
        self.run_name = run_name
        self.history: list[dict] = []
        self.best_success = -1.0
        self._next = eval_freq

    def _evaluate(self) -> dict:
        rec = MetricsRecorder(self.out_dir, policy=self.run_name)
        self.eval_env.trace_enabled = False
        for ep in range(self.n_episodes):
            obs, info = self.eval_env.reset(seed=10_000 + ep)
            done, ep_rew, last_info = False, 0.0, info
            while not done:
                action, _ = self.model.predict(obs, deterministic=True)
                obs, rew, term, trunc, last_info = self.eval_env.step(action)
                ep_rew += float(rew)
                done = term or trunc
            last_info.setdefault(
                "is_success",
                float(last_info.get("gate_index", 0) >= self.eval_env.cfg.n_gates),
            )
            rec.add(ep, last_info, ep_rew, seed=10_000 + ep)
        return rec.summary()

    def _on_step(self) -> bool:
        if self.num_timesteps < self._next:
            return True
        self._next = self.num_timesteps + self.eval_freq

        summary = self._evaluate()
        summary["timesteps"] = self.num_timesteps
        self.history.append(summary)

        self.logger.record("eval/success_rate", summary["success_rate"])
        self.logger.record("eval/collision_rate", summary["collision_rate"])
        self.logger.record("eval/mean_min_safety_margin_m",
                           summary["mean_min_safety_margin_m"] or 0.0)
        self.logger.record("eval/mean_arrival_time_s",
                           summary["mean_arrival_time_s"] or 0.0)
        self.logger.record("eval/mean_episode_cost", summary["mean_episode_cost"])

        margin = summary["mean_min_safety_margin_m"]
        arrival = summary["mean_arrival_time_s"]
        print(
            f"[eval ] step={self.num_timesteps:>8d}  "
            f"success={summary['success_rate']:.3f}  "
            f"collision={summary['collision_rate']:.3f}  "
            f"min_margin={'n/a' if margin is None else f'{margin:.3f}m'}  "
            f"arrival={'n/a' if arrival is None else f'{arrival:.2f}s'}  "
            f"cost={summary['mean_episode_cost']:.3f}"
        )

        if summary["success_rate"] > self.best_success:
            self.best_success = summary["success_rate"]
            self.checkpoint_dir.mkdir(parents=True, exist_ok=True)
            self.model.save(str(self.checkpoint_dir / f"{self.run_name}_best"))
        return True


# ==========================================================================
# main
# ==========================================================================


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Train a safe DRL navigator (PPO / PPO-Lagrangian, optional CBF filter)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--policy", choices=["ppo", "ppo_lag"], default="ppo_lag")
    p.add_argument("--use-cbf", action="store_true",
                   help="layer the CBF safety filter on top of the policy action")
    p.add_argument("--timesteps", type=int, default=400_000)
    p.add_argument("--n-envs", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--run-name", type=str, default=None)
    # Fine-tuning a warm-started policy needs a much gentler step size than
    # training from scratch: at 3e-4 the 0.875-success warm start was destroyed
    # within 20k steps (success fell to 0.0), whereas 5e-5 preserves and then
    # improves it (see logs/ for both runs).
    p.add_argument("--learning-rate", type=float, default=5e-5)
    p.add_argument("--n-steps", type=int, default=1024)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--n-epochs", type=int, default=10)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    # 0.0 for the fine-tune: an entropy bonus pushes the policy back towards
    # the wide initial action distribution, which is exactly what destabilises
    # a warm-started flight controller.
    p.add_argument("--ent-coef", type=float, default=0.0)
    p.add_argument("--normalize-reward", action="store_true",
                   help="wrap the env in VecNormalize(norm_reward=True)")
    p.add_argument("--init-from", type=str, default=None,
                   help="checkpoint .zip to warm-start the policy from "
                        "(produced by pretrain_bc.py).  All ablation rows use the "
                        "SAME warm start so that only the safety mechanism differs.")
    p.add_argument("--log-std-init", type=float, default=-1.0,
                   help="initial policy log-std. SB3's default of 0 gives actions "
                        "~N(0,1) over the full [-1,1] range, which is far too "
                        "violent for a quadrotor: the vehicle crashes within a "
                        "second and never discovers flight. -1.0 (std~0.37) "
                        "lets it explore gently around a hover.")
    p.add_argument("--cost-limit", type=float, default=0.6,
                   help="allowed mean EPISODIC constraint cost; the cost is "
                        "1{collision} + 0.5*(margin-encroachment rate), so this "
                        "is essentially 'at most a 60%% collision rate'")
    p.add_argument("--lambda-lr", type=float, default=0.03)
    p.add_argument("--lambda-max", type=float, default=8.0,
                   help="cap on the multiplier; an unbounded dual variable "
                        "swamps the task reward and destroys performance")
    p.add_argument("--eval-freq", type=int, default=25_000)
    p.add_argument("--eval-episodes", type=int, default=12)
    p.add_argument("--log-every", type=int, default=2048)
    p.add_argument("--max-steps", type=int, default=900, help="episode length cap")
    p.add_argument("--n-gates", type=int, default=3)
    p.add_argument("--n-static", type=int, default=7)
    p.add_argument("--n-dynamic", type=int, default=4)
    p.add_argument("--gui", action="store_true", help="render the PyBullet GUI")
    p.add_argument("--dummy-vec", action="store_true", help="force DummyVecEnv")
    p.add_argument("--device", default="auto")
    p.add_argument("--tag", default="", help="suffix for the run directory")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    paths = ensure_dirs()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_name = args.run_name or f"{args.policy}{'_cbf' if args.use_cbf else ''}_{stamp}"
    tb_dir = paths["tb"] / run_name

    # ---- environment configuration -----------------------------------
    overrides = dict(
        max_steps=args.max_steps,
        n_gates=args.n_gates,
        n_static=args.n_static,
        n_dynamic=args.n_dynamic,
        gui=args.gui,
        use_cbf=args.use_cbf,
        cost_limit=args.cost_limit,
    )

    banner = {
        "run_name": run_name,
        "policy": args.policy,
        "use_cbf": args.use_cbf,
        "timesteps": args.timesteps,
        "n_envs": args.n_envs,
        "cost_limit": args.cost_limit,
        "seed": args.seed,
    }

    with RunLogger(paths["logs"], name=f"train_{run_name}") as rl:
        try:
            rl.section(f"SafeDRL-DroneNav-ROS :: TRAINING  [{run_name}]")
            for k, v in banner.items():
                rl.log(f"{k:<14s} = {v}")
            rl.log(f"tensorboard dir = {tb_dir}")

            vec_env, vec_kind = build_vec_env(
                args.n_envs, args.seed, overrides, args.dummy_vec, args.normalize_reward
            )
            rl.log(f"vectorised environment : {vec_kind} x {vec_env.num_envs}")
            rl.log(f"observation space      : {vec_env.observation_space.shape}")
            rl.log(f"action space           : {vec_env.action_space.shape}")

            # ---- model -----------------------------------------------------
            common = dict(
                policy="MlpPolicy",
                env=vec_env,
                learning_rate=args.learning_rate,
                n_steps=args.n_steps,
                batch_size=args.batch_size,
                n_epochs=args.n_epochs,
                gamma=args.gamma,
                gae_lambda=args.gae_lambda,
                ent_coef=args.ent_coef,
                verbose=0,
                seed=args.seed,
                device=args.device,
                tensorboard_log=str(tb_dir),
                policy_kwargs=dict(
                    net_arch=dict(pi=[128, 128], vf=[128, 128]),
                    log_std_init=args.log_std_init,
                ),
            )

            if args.policy == "ppo_lag":
                from lagrangian_ppo import LagrangianPPO
                model = LagrangianPPO(
                    constrained=True,
                    cost_limit=args.cost_limit,
                    lambda_lr=args.lambda_lr,
                    lambda_max=args.lambda_max,
                    cost_net_arch=[128, 128],
                    **common,
                )
                rl.log("algorithm              : PPO-Lagrangian (cost critic + dual ascent)")
            else:
                from stable_baselines3 import PPO
                model = PPO(**common)
                rl.log("algorithm              : PPO (unconstrained baseline)")

            n_params = sum(p.numel() for p in model.policy.parameters())
            rl.log(f"policy parameters      : {n_params:,}")

            # ---- optional demonstration warm-start --------------------------
            if args.init_from:
                loaded = _load_policy_weights(model, Path(args.init_from))
                rl.log(f"warm-started policy from: {args.init_from} ({loaded} tensors)")
                rl.log("  (all ablation rows share this checkpoint, so the only "
                       "difference between them is the safety mechanism)")

            # ---- callbacks --------------------------------------------------
            eval_cfg = dict(overrides)
            eval_cfg.update(dict(gui=False, seed=args.seed + 777))
            eval_env = SafeNavEnv(EnvConfig(**eval_cfg))
            eval_cb = SafetyEvalCallback(
                eval_env, args.eval_freq, args.eval_episodes,
                paths["logs"], run_name, checkpoint_dir=paths["checkpoints"],
            )
            log_cb = TrainingLogCallback(args.log_every)

            rl.section("Training")
            t0 = time.time()
            model.learn(
                total_timesteps=args.timesteps,
                callback=[log_cb, eval_cb],
                tb_log_name="tb",
                progress_bar=False,
            )
            train_wall = time.time() - t0

            rl.section("Training finished")
            rl.log(f"wall-clock training time : {train_wall:.1f} s")
            rl.log(f"throughput               : {args.timesteps / max(train_wall, 1e-9):.0f} steps/s")
            rl.log(f"final lambda             : {getattr(model, 'lagrange', None) and model.lagrange.lam}")

            ckpt = paths["checkpoints"] / f"{run_name}_final"
            model.save(str(ckpt))
            rl.log(f"saved final model        : {ckpt}.zip")

            # ---- eval history dump ------------------------------------------
            (paths["logs"] / f"{run_name}_eval_history.json").write_text(
                json.dumps(eval_cb.history, indent=2, default=str), encoding="utf-8"
            )

            write_run_manifest(
                paths,
                {
                    "run_name": run_name,
                    "kind": "train",
                    "banner": banner,
                    "train_wall_s": train_wall,
                    "vector_env": vec_kind,
                    "n_policy_params": n_params,
                    "tensorboard": str(tb_dir),
                },
            )
            print(f"\nRUN_NAME={run_name}")
            print(f"TB_DIR={tb_dir}")
            print(f"MODEL={ckpt}.zip")
        except Exception:
            print("\n!!! training aborted !!!")
            traceback.print_exc()
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
