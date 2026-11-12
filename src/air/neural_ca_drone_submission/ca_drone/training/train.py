"""Supervised training loops for the four neural modules.

Everything runs on plain NumPy (see :mod:`ca_drone.nn.autograd`), which keeps the
reference implementation reproducible on a laptop without CUDA.  Each loop
returns a history dict that is later used to draw the learning curves in the
report.
"""

from __future__ import annotations

import json
import os
import time
from typing import Dict, List, Tuple

import numpy as np

from ..models.control import ControlNet, MAX_SPEED
from ..models.e2e import E2ENet
from ..models.perception import PerceptionNet, normalize_depth
from ..models.planner import NeuralHeuristic
from ..nn.autograd import Tensor, bce_with_logits, mse
from ..nn.optim import Adam, iterate_batches


def _split(n: int, val_frac: float = 0.15, seed: int = 0) -> Tuple[np.ndarray, np.ndarray]:
    idx = np.arange(n)
    np.random.default_rng(seed).shuffle(idx)
    k = max(1, int(n * val_frac))
    return idx[k:], idx[:k]


def _save_history(history: Dict, path: str):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(history, f, indent=2)


def train_perception(data: Dict[str, np.ndarray], epochs: int = 4, batch: int = 8,
                     lr: float = 2e-3, seed: int = 0, verbose: bool = True) -> Dict:
    depth, mask = data["depth"], data["mask"].astype(np.float32)
    n = len(depth)
    tr, va = _split(n, 0.15, seed)
    model = PerceptionNet()
    opt = Adam(model.parameters(), lr=lr, weight_decay=1e-5)
    rng = np.random.default_rng(seed + 1)
    pos = float(mask[tr].sum())
    neg = float(mask[tr].size - pos)
    pos_weight = float(np.clip(neg / max(pos, 1.0), 1.0, 20.0))
    history = {"module": "perception", "epochs": [], "params": model.num_params(),
               "pos_weight": pos_weight, "train_n": int(len(tr)), "val_n": int(len(va))}
    t0 = time.time()
    for ep in range(epochs):
        losses = []
        for idx in iterate_batches(len(tr), batch, rng):
            xb = normalize_depth(depth[tr[idx]])
            yb = mask[tr[idx]][:, None]
            opt.zero_grad()
            logits = model(Tensor(xb))
            w = np.where(yb > 0.5, pos_weight, 1.0).astype(np.float32)
            loss = bce_with_logits(logits, yb, weight=w, reduce=True)
            loss.backward()
            opt.step()
            losses.append(float(loss.data))
        # validation
        probs = []
        for start in range(0, len(va), 8):
            sl = va[start:start + 8]
            out = model(Tensor(normalize_depth(depth[sl]))).data[:, 0]
            probs.append(1.0 / (1.0 + np.exp(-np.clip(out, -60, 60))))
        prob = np.concatenate(probs, 0)
        tgt = mask[va] > 0.5
        pred = prob > 0.5
        inter = float((pred & tgt).sum())
        union = float((pred | tgt).sum())
        iou = inter / max(union, 1.0)
        dice = 2 * inter / max(float(pred.sum() + tgt.sum()), 1.0)
        rec = {"epoch": ep + 1, "train_loss": float(np.mean(losses)),
               "val_iou": iou, "val_dice": dice, "seconds": round(time.time() - t0, 1)}
        history["epochs"].append(rec)
        if verbose:
            print(f"[perception] epoch {ep + 1}/{epochs} loss={rec['train_loss']:.4f} "
                  f"IoU={iou:.3f} Dice={dice:.3f} ({rec['seconds']}s)")
    history["total_seconds"] = round(time.time() - t0, 1)
    return {"model": model, "history": history}


def train_planner(data: Dict[str, np.ndarray], epochs: int = 12, batch: int = 128,
                  lr: float = 2e-3, seed: int = 0, verbose: bool = True) -> Dict:
    X, Y = data["x"], data["y"]
    tr, va = _split(len(X), 0.15, seed)
    model = NeuralHeuristic()
    opt = Adam(model.parameters(), lr=lr, weight_decay=1e-5)
    rng = np.random.default_rng(seed + 2)
    history = {"module": "planner", "epochs": [], "params": model.num_params()}
    t0 = time.time()
    for ep in range(epochs):
        losses = []
        for idx in iterate_batches(len(tr), batch, rng):
            opt.zero_grad()
            out = model(Tensor(X[tr[idx]]))
            loss = mse(out, Y[tr[idx]], reduce=True)
            loss.backward()
            opt.step()
            losses.append(float(loss.data))
        pred = model(Tensor(X[va])).data
        mae = float(np.mean(np.abs(pred - Y[va])) * 50.0)
        var = float(np.var(Y[va]))
        r2 = 1.0 - float(np.mean((pred - Y[va]) ** 2)) / max(var, 1e-8)
        rec = {"epoch": ep + 1, "train_loss": float(np.mean(losses)),
               "val_mae_m": mae, "val_r2": r2, "seconds": round(time.time() - t0, 1)}
        history["epochs"].append(rec)
        if verbose:
            print(f"[planner] epoch {ep + 1}/{epochs} loss={rec['train_loss']:.5f} "
                  f"MAE={mae:.2f}m R2={r2:.3f} ({rec['seconds']}s)")
    history["total_seconds"] = round(time.time() - t0, 1)
    return {"model": model, "history": history}


def train_control(data: Dict[str, np.ndarray], epochs: int = 60, batch: int = 128,
                  lr: float = 2e-3, seed: int = 0, verbose: bool = True) -> Dict:
    X, Y = data["x"], data["y"]
    tr, va = _split(len(X), 0.15, seed)
    model = ControlNet()
    opt = Adam(model.parameters(), lr=lr, weight_decay=1e-5)
    rng = np.random.default_rng(seed + 3)
    history = {"module": "control", "epochs": [], "params": model.num_params()}
    t0 = time.time()
    for ep in range(epochs):
        losses = []
        for idx in iterate_batches(len(tr), batch, rng):
            opt.zero_grad()
            out = model(Tensor(X[tr[idx]]))
            loss = mse(out, Y[tr[idx]], reduce=True)
            loss.backward()
            opt.step()
            losses.append(float(loss.data))
        pred = model(Tensor(X[va])).data
        mae = float(np.mean(np.abs(pred - Y[va])))
        ang = float(np.mean(np.degrees(np.arccos(np.clip(
            np.sum(pred * Y[va], 1) / (np.linalg.norm(pred, axis=1) * np.linalg.norm(Y[va], axis=1) + 1e-6), -1, 1)))))
        rec = {"epoch": ep + 1, "train_loss": float(np.mean(losses)),
               "val_mae_ms": mae, "val_angle_deg": ang, "seconds": round(time.time() - t0, 1)}
        history["epochs"].append(rec)
        if verbose and (ep % 5 == 0 or ep == epochs - 1):
            print(f"[control] epoch {ep + 1}/{epochs} loss={rec['train_loss']:.5f} "
                  f"MAE={mae:.3f}m/s angle={ang:.1f}deg ({rec['seconds']}s)")
    history["total_seconds"] = round(time.time() - t0, 1)
    return {"model": model, "history": history}


def train_e2e(data: Dict[str, np.ndarray], epochs: int = 12, batch: int = 16,
              lr: float = 1.5e-3, seed: int = 0, verbose: bool = True) -> Dict:
    X, G, Y = data["depth"], data["goal"], data["cmd"]
    tr, va = _split(len(X), 0.15, seed)
    model = E2ENet()
    opt = Adam(model.parameters(), lr=lr, weight_decay=1e-5)
    rng = np.random.default_rng(seed + 4)
    history = {"module": "e2e", "epochs": [], "params": model.num_params()}
    t0 = time.time()
    for ep in range(epochs):
        losses = []
        for idx in iterate_batches(len(tr), batch, rng):
            opt.zero_grad()
            out = model(Tensor(X[tr[idx]][:, None]), Tensor(G[tr[idx]]))
            loss = mse(out, Y[tr[idx]], reduce=True)
            loss.backward()
            opt.step()
            losses.append(float(loss.data))
        pred = model(Tensor(X[va][:, None]), Tensor(G[va])).data
        mae = float(np.mean(np.abs(pred - Y[va])))
        rec = {"epoch": ep + 1, "train_loss": float(np.mean(losses)),
               "val_mae_ms": mae, "seconds": round(time.time() - t0, 1)}
        history["epochs"].append(rec)
        if verbose:
            print(f"[e2e] epoch {ep + 1}/{epochs} loss={rec['train_loss']:.5f} "
                  f"MAE={mae:.3f}m/s ({rec['seconds']}s)")
    history["total_seconds"] = round(time.time() - t0, 1)
    return {"model": model, "history": history}
