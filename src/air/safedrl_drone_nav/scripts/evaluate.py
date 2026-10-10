#!/usr/bin/env python3
"""
Evaluation and figure generation for SafeDRL-DroneNav-ROS.

Runs a trained checkpoint over a fixed evaluation episode set, records a full
per-step trace, and writes every artefact the report needs:

``logs/eval_metrics.json``            structured metrics for this policy
``logs/eval_summary.csv``             one-row summary table
``logs/eval_metrics_episodes.csv``    per-episode detail
``logs/traces/<run>_trace.npz``       raw per-step telemetry
``logs/terminal_snippet_<run>.txt``   verbatim timestamped terminal excerpt
``docs/figures/*.png``                trajectory, state response, margins

Usage::

    python scripts/evaluate.py --model checkpoints/ppo_lag_cbf_final.zip --use-cbf \\
        --run-name ppo_lag_cbf --episodes 20 --figures
"""

from __future__ import annotations

import argparse
import io
import json
import sys
import time
from contextlib import redirect_stdout
from datetime import datetime
from pathlib import Path

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from run_logging import (MetricsRecorder, RunLogger, ensure_dirs,      # noqa: E402
                         snapshot_terminal, write_run_manifest)
from safe_nav_env import EnvConfig, SafeNavEnv                          # noqa: E402
import figures as F                                                     # noqa: E402


# --------------------------------------------------------------------------


def load_model(path: Path, algo: str = "auto"):
    """Load a checkpoint, preferring the custom Lagrangian class."""
    from stable_baselines3 import PPO
    if algo in ("auto", "ppo_lag"):
        try:
            import lagrangian_ppo  # noqa: F401
            return PPO.load(str(path), device="cpu")
        except Exception:  # noqa: BLE001
            pass
    return PPO.load(str(path), device="cpu")


def rollout(model, env: SafeNavEnv, seed: int, stochastic: bool = False):
    """One episode with a full per-step trace."""
    env.trace_enabled = True
    obs, info = env.reset(seed=seed)
    done, ep_rew, n = False, 0.0, 0
    last = info
    while not done:
        action, _ = model.predict(obs, deterministic=not stochastic)
        obs, r, term, trunc, last = env.step(action)
        ep_rew += float(r)
        done = term or trunc
        n += 1
    trace = env.trace
    out = {
        "t": np.array([s["t"] for s in trace]),
        "pos": np.array([s["pos"] for s in trace]),
        "vel": np.array([s["vel"] for s in trace]),
        "rpy": np.array([s["rpy"] for s in trace]),
        "v_des": np.array([s["v_des"] for s in trace]),
        "v_safe": np.array([s["v_safe"] for s in trace]),
        "action": np.array([s["action"] for s in trace]),
        "rpm": np.array([s["rpm"] for s in trace]),
        "margin": np.array([s["margin"] for s in trace]),
        "contact_clearance": np.array([s["contact_clearance"] for s in trace]),
        "dist_gate": np.array([s["dist_gate"] for s in trace]),
        "wind": np.array([s["wind"] for s in trace]),
        "reward": np.array([s["reward"] for s in trace]),
        "cost": np.array([s["cost"] for s in trace]),
        "gate_idx": np.array([s["gate_idx"] for s in trace]),
    }
    return out, last, ep_rew


def record_dynamic_paths(env: SafeNavEnv, trace: dict) -> dict:
    """Replay the analytic obstacle motion to draw obstacle paths in 3-D."""
    out = {}
    for i, ob in enumerate(env.obstacle_states()):
        if ob["kind"] != "dynamic":
            continue
        src = env.obstacles[i]
        t = trace["t"]
        centres = np.array([
            src["centre0"] + src["axis"] * src["amp"] * np.sin(src["omega"] * ti + src["phase"])
            for ti in t
        ])
        out[i] = centres
    return out


# --------------------------------------------------------------------------


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--model", required=True, help="checkpoint .zip")
    p.add_argument("--run-name", default=None)
    p.add_argument("--episodes", type=int, default=20)
    p.add_argument("--seed0", type=int, default=20_000)
    p.add_argument("--use-cbf", action="store_true")
    p.add_argument("--n-gates", type=int, default=3)
    p.add_argument("--n-static", type=int, default=7)
    p.add_argument("--n-dynamic", type=int, default=4)
    p.add_argument("--max-steps", type=int, default=900)
    p.add_argument("--gui", action="store_true")
    p.add_argument("--stochastic", action="store_true")
    p.add_argument("--figures", action="store_true", default=True)
    p.add_argument("--no-figures", dest="figures", action="store_false")
    p.add_argument("--trace-episode", type=int, default=0,
                   help="which episode index to use as the figure showcase")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    paths = ensure_dirs()
    model_path = Path(args.model)
    run = args.run_name or model_path.stem

    overrides = dict(
        gui=args.gui, use_cbf=args.use_cbf, max_steps=args.max_steps,
        n_gates=args.n_gates, n_static=args.n_static, n_dynamic=args.n_dynamic,
    )

    with RunLogger(paths["logs"], name=f"eval_{run}") as rl:
        rl.section(f"SafeDRL-DroneNav-ROS :: EVALUATION  [{run}]")
        rl.log(f"model      = {model_path}")
        rl.log(f"use_cbf    = {args.use_cbf}")
        rl.log(f"episodes   = {args.episodes}  seeds {args.seed0}..{args.seed0 + args.episodes - 1}")
        rl.log(f"deterministic = {not args.stochastic}")

        model = load_model(model_path)
        env = SafeNavEnv(EnvConfig(**overrides))
        rl.log(f"gates      = {np.round(env.gates, 2).tolist()}")
        # The obstacle set is built inside reset(), so inspect it only after a
        # throwaway reset - reading env.obstacles at construction time reports 0.
        env.reset(seed=args.seed0)
        n_static = sum(o["kind"] == "static" for o in env.obstacles)
        n_dynamic = sum(o["kind"] == "dynamic" for o in env.obstacles)
        rl.log(f"obstacles  = {len(env.obstacles)} ({n_static} static, {n_dynamic} dynamic)")

        rec = MetricsRecorder(paths["logs"], policy=run)
        traces, showcase = [], None
        gate_history = []

        buf = io.StringIO()
        t0 = time.time()
        with redirect_stdout(buf):
            for ep in range(args.episodes):
                seed = args.seed0 + ep
                trace, last, ep_rew = rollout(model, env, seed, args.stochastic)
                rec.add(ep, last, ep_rew, seed=seed)
                traces.append(trace)
                if ep == args.trace_episode:
                    showcase = trace
                    showcase_obstacles = env.obstacle_states()
                    showcase_paths = record_dynamic_paths(env, trace)
                    showcase_gates = env.gates.copy()
                # retain per-episode gate progress for the summary figure
                gate_history.append((trace["t"], trace["gate_idx"]))
                status = ("SUCCESS" if last["is_success"]
                          else "COLLISION" if last["is_collision"] else "TIMEOUT")
                # report the *episode minimum* clearance, which is the quantity
                # the safety claim is stated in
                print(f"  ep{ep:3d} seed={seed:6d} {status:9s} "
                      f"gates={last['gate_index']}/{args.n_gates} "
                      f"t={last['time']:5.2f}s ret={ep_rew:+8.2f} "
                      f"min_margin={last['min_margin']:+.3f}m "
                      f"min_clearance={last['min_contact_clearance']:+.3f}m "
                      f"cost={last['episode_cost']:.2f}")
        wall = time.time() - t0
        print(buf.getvalue())

        # Per-run filenames.  The default stem would make every evaluation
        # overwrite logs/eval_metrics.json, silently discarding the evidence
        # for all but the last policy.
        summary = rec.write(
            extra={
                "model": str(model_path),
                "use_cbf": bool(args.use_cbf),
                "seed0": args.seed0,
                "evaluation_wall_s": wall,
                "deterministic": not args.stochastic,
            },
            stem=f"eval_metrics_{run}",
        )
        rec.print_summary()

        # ---- traces -----------------------------------------------------
        np.savez_compressed(
            paths["traces"] / f"{run}_traces.npz",
            **{f"ep{k}_{name}": v for k, tr in enumerate(traces) for name, v in tr.items()},
        )
        single = {"pos": showcase["pos"], "t": showcase["t"], "vel": showcase["vel"],
                  "rpy": showcase["rpy"], "v_des": showcase["v_des"],
                  "v_safe": showcase["v_safe"], "rpm": showcase["rpm"],
                  "margin": showcase["margin"],
                  "contact_clearance": showcase["contact_clearance"],
                  "dist_gate": showcase["dist_gate"], "wind": showcase["wind"],
                  "gate_idx": showcase["gate_idx"], "action": showcase["action"]}
        np.savez_compressed(paths["traces"] / f"{run}_trace_ep{args.trace_episode}.npz",
                            **single)
        (paths["logs"] / f"{run}_obstacles.json").write_text(
            json.dumps(
                [{"kind": o["kind"], "centre": [float(x) for x in o["centre"]],
                  "radius": float(o["radius"])} for o in showcase_obstacles],
                indent=2,
            ), encoding="utf-8")

        # ---- terminal evidence -------------------------------------------
        snippet = snapshot_terminal(
            buf.getvalue(), paths["logs"] / f"terminal_snippet_{run}.txt",
            title=f"evaluation of {run} ({args.episodes} episodes)",
        )
        rl.log(f"terminal snippet : {snippet}")

        # ---- figures ------------------------------------------------------
        if args.figures:
            rl.section("Figures")
            F.plot_trajectory_3d(
                showcase, showcase_gates, showcase_obstacles,
                paths["figures"] / f"trajectory_3d_{run}.png",
                title=f"3-D flight trajectory - {run}\n"
                      f"success {summary['success_rate']:.0%}, "
                      f"min margin {summary['mean_min_safety_margin_m']:+.3f} m",
                obstacle_paths=showcase_paths,
            )
            F.plot_state_response(
                showcase, paths["figures"] / f"state_response_{run}.png",
                title=f"State time-domain response - {run}",
            )
            F.plot_gate_progress(
                showcase, args.n_gates, paths["figures"] / f"gate_progress_{run}.png",
                title=f"Gate traversal progress - {run}",
            )

        write_run_manifest(paths, {
            "run_name": run, "kind": "evaluate", "model": str(model_path),
            "use_cbf": bool(args.use_cbf), "episodes": args.episodes,
            "summary": summary,
        })
        print(f"\nEVAL_RUN={run}")
        print(f"SUCCESS_RATE={summary['success_rate']:.4f}")
        print(f"COLLISION_RATE={summary['collision_rate']:.4f}")
        print(f"WORST_CLEARANCE={summary['worst_contact_clearance_m']}")
    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
