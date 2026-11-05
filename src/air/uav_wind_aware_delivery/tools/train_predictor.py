#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练落点预测 MLP，并保存模型与指标。"""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src"))
from uav_wind_aware_delivery.ballistics import simulate_drop  # noqa: E402
from uav_wind_aware_delivery.nn import ImpactMLP  # noqa: E402


def make_dataset(n=6000, seed=20261104):
    rng = np.random.default_rng(seed)
    h = rng.uniform(2.0, 4.2, n)
    vx = rng.uniform(-0.8, 0.8, n)
    vy = rng.uniform(-0.8, 0.8, n)
    vz = rng.uniform(-0.4, 0.4, n)
    wx = rng.uniform(-5.0, 5.0, n)
    wy = rng.uniform(-3.0, 3.0, n)
    X = np.column_stack([h, vx, vy, vz, wx, wy])
    y = np.array([simulate_drop(h[i], vx[i], vy[i], vz[i], np.array([wx[i], wy[i]]))
                  for i in range(n)])
    return X, y


def main():
    X, y = make_dataset()
    n_val = 2000
    Xtr, ytr = X[:-n_val], y[:-n_val]
    Xva, yva = X[-n_val:], y[-n_val:]
    model = ImpactMLP(seed=7)
    report = model.fit(Xtr, ytr, epochs=110, lr=0.007, batch=256, val_ratio=0.0)
    pred = model.predict(Xva)
    rmse = float(np.sqrt(np.mean((pred - yva) ** 2)))
    mae = float(np.mean(np.abs(pred - yva)))
    zero_rmse = float(np.sqrt(np.mean(yva ** 2)))
    out = {
        "model": "ImpactMLP(6-64-64-2)",
        "samples": int(len(X)),
        "train_rmse_m": round(report.train_rmse, 5),
        "val_rmse_m": round(rmse, 5),
        "val_mae_m": round(mae, 5),
        "zero_baseline_rmse_m": round(zero_rmse, 5),
        "improvement_pct": round((1.0 - rmse / zero_rmse) * 100.0, 2),
        "epochs": report.epochs,
    }
    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    path = os.path.join(ROOT, "models", "impact_mlp.npz")
    model.save(path, meta=out)
    print(json.dumps(out, ensure_ascii=False, indent=2))
    with open(os.path.join(ROOT, "results", "predictor_metrics.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
