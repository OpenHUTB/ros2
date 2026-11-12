#!/usr/bin/env python3
"""Train the IK network: target position -> joint angles.

Input : target position in the workspace (normalised x, y, radius, sin, cos)
Output: shoulder / elbow joint angles (elbow-up analytic IK labels).
Target: mean angular error <= 2 degrees.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from msk_arm_nn import config as C
from msk_arm_nn.datasets import ik_solve, q_des_for_target
from msk_arm_nn.nn.mlp import MLP

Q1_SCALE = max(abs(C.Q1_MIN), abs(C.Q1_MAX))
Q2_SCALE = max(abs(C.Q2_MIN), abs(C.Q2_MAX))


def target_features(x: float, y: float) -> np.ndarray:
    r = math.hypot(x, y)
    ang = math.atan2(y, x)
    return np.array([x / 0.70, y / 0.70, r / 0.70, math.sin(ang), math.cos(ang)],
                    dtype=np.float64)


def sample_targets(n: int, rng: np.random.Generator):
    """Sample workspace targets, including slightly out-of-reach ones.

    Out-of-reach targets are labelled with the saturated (fully extended)
    solution, exactly like the expert controller, so the IK network also learns
    what to do when the target cannot be reached (the ``draw`` scene contains
    two such points).
    """
    xs, ys = [], []
    tries = 0
    while len(xs) < n and tries < n * 40:
        tries += 1
        r = float(rng.uniform(0.04, C.L1 + C.L2 + 0.06))
        ang = rng.uniform(-0.35 * math.pi, 0.95 * math.pi)
        x, y = r * math.cos(ang), r * math.sin(ang)
        if not (C.WORK_X[0] + 0.01 <= x <= C.WORK_X[1] - 0.01):
            continue
        if not (C.WORK_Y[0] + 0.01 <= y <= C.WORK_Y[1] - 0.01):
            continue
        xs.append(x)
        ys.append(y)
    return np.asarray(xs), np.asarray(ys)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=12000)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--lr", type=float, default=2e-3)
    parser.add_argument("--hidden", type=int, default=96)
    parser.add_argument("--val-fraction", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--models-dir", default=C.MODELS_DIR)
    args = parser.parse_args()

    t0 = time.time()
    rng = np.random.default_rng(args.seed)
    x, y = sample_targets(int(args.samples), rng)
    # include the real scene targets so the validation set is task-relevant
    extra = []
    for name in C.SCENES:
        from msk_arm_nn.scenes import get_scene
        extra.extend(list(get_scene(name).targets))
    for (tx, ty) in extra:
        x = np.append(x, tx)
        y = np.append(y, ty)
    F = np.stack([target_features(a, b) for a, b in zip(x, y)], axis=0)
    Q = np.zeros((len(x), 2), dtype=np.float64)
    reachable = np.zeros(len(x), dtype=bool)
    for i, (a, b) in enumerate(zip(x, y)):
        Q[i] = q_des_for_target(float(a), float(b))
        reachable[i] = math.hypot(float(a), float(b)) <= C.L1 + C.L2
    F = F
    perm = rng.permutation(len(F))
    n_val = max(1, int(round(len(F) * args.val_fraction)))
    va, tr = perm[:n_val], perm[n_val:]
    Y = np.stack([Q[:, 0] / Q1_SCALE, Q[:, 1] / Q2_SCALE], axis=1)
    print("samples train %d / val %d (reachable targets only)"
          % (len(tr), len(va)))

    net = MLP([F.shape[1], args.hidden, args.hidden, 2],
              activation="relu", seed=args.seed, lr=args.lr)
    rng2 = np.random.default_rng(args.seed + 1)
    for epoch in range(1, args.epochs + 1):
        if epoch == max(2, int(round(0.6 * args.epochs))):
            net.set_lr(args.lr * 0.35)
        order = rng2.permutation(len(tr))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = tr[order[start:start + args.batch]]
            losses.append(net.train_step(F[idx], Y[idx], loss="mse"))
        if epoch == 1 or epoch % 10 == 0 or epoch == args.epochs:
            pred = net.predict(F[va])
            err = np.abs(pred - Y[va])
            deg = float(np.mean(err) * 0.5 * (Q1_SCALE + Q2_SCALE)) * 180.0 / math.pi
            print("epoch %3d  train_mse %.6f  val_mean_deg %.3f"
                  % (epoch, float(np.mean(losses)), deg))

    pred = net.predict(F[va])
    q_pred = np.stack([pred[:, 0] * Q1_SCALE, pred[:, 1] * Q2_SCALE], axis=1)
    q_true = Q[va]
    err = np.abs(q_pred - q_true)
    err_deg = err * 180.0 / math.pi
    mae_deg = float(err_deg.mean())
    max_deg = float(err_deg.max())
    # separated metrics: truly reachable targets vs saturated solutions
    va_reach = reachable[va]
    reachable_mae_deg = (float(err_deg[va_reach].mean())
                         if bool(va_reach.any()) else mae_deg)
    # real scene targets
    scene_errs = []
    for name in C.SCENES:
        from msk_arm_nn.scenes import get_scene
        for (tx, ty) in get_scene(name).targets:
            ik = ik_solve(float(tx), float(ty))
            if ik is None:
                continue
            p = net.predict(target_features(tx, ty))
            qp = np.array([p[0] * Q1_SCALE, p[1] * Q2_SCALE])
            scene_errs.append(float(np.abs(qp - np.asarray(ik)).mean()
                                    * 180.0 / math.pi))
    scene_mae = float(np.mean(scene_errs)) if scene_errs else mae_deg
    seconds = time.time() - t0
    metrics = {
        "model": "ik_mlp",
        "task": "inverse kinematics: target position -> (q1, q2), elbow-up",
        "architecture": [int(F.shape[1]), args.hidden, args.hidden, 2],
        "input": "target (x, y, r, sin, cos), normalised",
        "output": "(q1/Q1_SCALE, q2/Q2_SCALE)",
        "samples": int(len(F)),
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "val_samples": int(len(va)),
        "ik_mae_deg": round(mae_deg, 4),
        "ik_max_deg": round(max_deg, 4),
        "ik_mae_rad": round(math.radians(mae_deg), 5),
        "scene_targets_mae_deg": round(scene_mae, 4),
        "reachable_targets_mae_deg": round(reachable_mae_deg, 4),
        "target_deg": 2.0,
        "seconds": round(seconds, 2),
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "ik_mlp.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_ik.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("IK network: mean error %.3f deg (target <= 2.0), scene targets "
          "%.3f deg | %.1f s" % (mae_deg, scene_mae, seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
