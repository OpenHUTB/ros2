#!/usr/bin/env python3
"""
Assemble the report figures from the artefacts left behind by real runs.

This script deliberately reads only *evidence files* - the TensorBoard event
files under ``logs/tb_logs/`` and the ``logs/eval_metrics*.json`` written by
``evaluate.py`` - so every figure in the README can be traced back to a run
that actually happened.

Outputs (all 200 dpi, into ``docs/figures/``):

    convergence_<policy>.png        training curves per policy
    training_dashboard_<policy>.png reward / cost / lambda / success
    ablation_<tag>.png              grouped bars across policies
    margins_<tag>.png               per-episode min-margin distributions
    metrics_table_<tag>.png         the metrics table rendered as a figure
    convergence_all.png             all policies overlaid

Usage::

    python scripts/make_report_figures.py \\
        --policy ppo_baseline --policy ppo_lagrangian --policy ppo_lag_cbf \\
        --tag ablation
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import figures as F                       # noqa: E402
from run_logging import ensure_dirs        # noqa: E402


# --------------------------------------------------------------------------
# TensorBoard reading
# --------------------------------------------------------------------------

TB_TAGS = {
    "ep_rew_mean": ["rollout/ep_rew_mean"],
    "ep_len_mean": ["rollout/ep_len_mean"],
    "policy_gradient_loss": ["train/policy_gradient_loss"],
    "value_loss": ["train/value_loss"],
    "entropy_loss": ["train/entropy_loss"],
    "lambda": ["constraint/lambda"],
    "mean_episode_cost": ["constraint/mean_episode_cost"],
    "mean_step_cost": ["constraint/mean_step_cost"],
    "cost_value_loss": ["train/cost_value_loss"],
    "success_rate": ["eval/success_rate", "rollout/success_rate"],
    "eval_collision_rate": ["eval/collision_rate"],
    "eval_min_margin": ["eval/mean_min_safety_margin_m"],
}


def read_tb(tb_root: Path, run: str, tags: dict | None = None) -> dict:
    """Extract scalar series from the newest event directory of ``run``."""
    tags = tags or TB_TAGS
    run_dir = tb_root / run
    if not run_dir.exists():
        print(f"  [warn] no tensorboard dir for {run}: {run_dir}")
        return {}

    # SB3 appends its own suffix (e.g. tb_1) when the name already exists
    candidates = sorted([p for p in run_dir.iterdir() if p.is_dir()],
                        key=lambda p: p.stat().st_mtime, reverse=True)
    if not candidates:
        print(f"  [warn] {run_dir} has no event subdirectories")
        return {}

    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except ImportError:
        print("  [warn] tensorboard not installed; skipping convergence figures")
        return {}

    series: dict[str, dict] = {}
    for event_dir in candidates:
        acc = EventAccumulator(str(event_dir), size_guidance={"scalars": 0})
        try:
            acc.Reload()
        except Exception as exc:  # noqa: BLE001
            print(f"  [warn] failed to read {event_dir}: {exc}")
            continue
        available = set(acc.Tags().get("scalars", []))
        for key, aliases in tags.items():
            if key in series:
                continue
            for alias in aliases:
                if alias in available:
                    events = acc.Scalars(alias)
                    series[key] = {
                        "step": [e.step for e in events],
                        "value": [e.value for e in events],
                    }
                    break
    return series


def read_metrics(logs_dir: Path, run: str) -> dict | None:
    p = logs_dir / f"eval_metrics_{run}.json"
    if p.exists():
        return json.loads(p.read_text(encoding="utf-8"))
    p = logs_dir / f"eval_metrics.json"
    if p.exists():
        data = json.loads(p.read_text(encoding="utf-8"))
        return data
    return None


def read_episodes(logs_dir: Path, run: str) -> dict | None:
    p = logs_dir / f"eval_metrics_{run}_episodes.csv"
    if p.exists():
        import csv
        rows = list(csv.DictReader(p.open(encoding="utf-8")))
        out: dict[str, list] = {}
        for k in rows[0]:
            vals = []
            for r in rows:
                try:
                    vals.append(float(r[k]))
                except (TypeError, ValueError):
                    vals.append(float("nan"))
            out[k] = vals
        return out
    return None


# --------------------------------------------------------------------------


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--policy", action="append", default=[],
                   help="run name; repeat for several (order = legend order)")
    p.add_argument("--tag", default="ablation")
    p.add_argument("--cost-limit", type=float, default=2.0)
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    paths = ensure_dirs()
    runs = args.policy or []
    if not runs:
        # discover every run that has evaluation metrics
        runs = sorted(p.name[len("eval_metrics_"):-len(".json")]
                      for p in paths["logs"].glob("eval_metrics_*.json"))
        print(f"  discovered runs: {runs}")

    print("\n== convergence ==")
    all_series = {}
    for run in runs:
        s = read_tb(paths["tb"], run)
        if not s:
            continue
        all_series[run] = s
        F.plot_convergence(
            {k: v for k, v in s.items()
             if k in ("ep_rew_mean", "ep_len_mean", "value_loss", "entropy_loss")},
            paths["figures"] / f"convergence_{run}.png",
            title=f"Training convergence - {run}",
        )
        dash = dict(s)
        dash["_cost_limit"] = args.cost_limit
        F.plot_training_dashboard(
            dash, paths["figures"] / f"training_dashboard_{run}.png",
            title=f"PPO-Lagrangian training dashboard - {run}",
        )

    if all_series:
        overlay = {}
        for run, s in all_series.items():
            for key in ("ep_rew_mean", "success_rate"):
                if key in s:
                    overlay[f"{run} :: {key}"] = s[key]
        if overlay:
            F.plot_convergence(
                overlay, paths["figures"] / "convergence_all.png",
                title="Training convergence - all policies", smooth=21,
            )

    print("\n== ablation ==")
    table = {}
    margins = {}
    for run in runs:
        m = read_metrics(paths["logs"], run)
        if not m:
            print(f"  [warn] no eval_metrics for {run}")
            continue
        m = dict(m)
        m["policy"] = run
        table[run] = m
        ep = read_episodes(paths["logs"], run)
        if ep and "min_contact_clearance" in ep:
            margins[run] = ep["min_contact_clearance"]

    if table:
        F.plot_ablation(table, paths["figures"] / f"ablation_{args.tag}.png",
                        title=f"Ablation: safety vs. task performance ({args.tag})")
        F.plot_metric_table(table, paths["figures"] / f"metrics_table_{args.tag}.png")
        # numeric sidecar so the README table can be generated/checked
        (paths["logs"] / f"{args.tag}_table.json").write_text(
            json.dumps(table, indent=2, ensure_ascii=False), encoding="utf-8"
        )
    if margins:
        F.plot_margin_distribution(
            margins, paths["figures"] / f"margins_{args.tag}.png",
            title=f"Per-episode minimum contact clearance ({args.tag})",
        )

    print(f"\nFigures written to {paths['figures']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
