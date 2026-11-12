"""train.py —— 训练深度图姿态估计网络"""
from __future__ import annotations
import os
import numpy as np
from .nn import DepthPoseMLP


def _rmse_deg(pred, target):
    """按角度（度）算 RMS 误差，比 MSE（rad²）直观：8° 的 RMS 对应 MSE≈0.0195。"""
    err = np.degrees(pred - target)
    return float(np.sqrt(np.mean(err**2)))


def train(dataset_path, out_path, input_dim=256, hidden1=128, hidden2=64,
          output_dim=3, epochs=80, batch_size=64, lr=0.005, seed=0, verbose=True):
    data = np.load(dataset_path)
    X, y = data["X"].astype(np.float64), data["y"].astype(np.float64)
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(y))
    X, y = X[idx], y[idx]
    n_val = max(1, int(len(y)*0.2))
    X_val, y_val = X[:n_val], y[:n_val]
    X_tr, y_tr = X[n_val:], y[n_val:]
    model = DepthPoseMLP(input_dim, hidden1, hidden2, output_dim, seed=seed)
    log = {"val_mse": [], "train_mse": [], "val_rmse_deg": []}
    best_mse, best_params, best_epoch = float("inf"), None, -1
    for ep in range(epochs):
        # 余弦退火：前期大步收敛，后期小步精修，避免末端在最优解附近来回抖
        cur_lr = lr * 0.5 * (1.0 + np.cos(np.pi * ep / epochs))
        perm = rng.permutation(len(y_tr))
        X_tr, y_tr = X_tr[perm], y_tr[perm]
        losses = []
        for i in range(0, len(y_tr), batch_size):
            losses.append(model.train_step(X_tr[i:i+batch_size], y_tr[i:i+batch_size], lr=cur_lr))
        val_pred = model.predict(X_val)
        val_mse = float(np.mean((val_pred - y_val)**2))
        log["train_mse"].append(float(np.mean(losses)))
        log["val_mse"].append(val_mse)
        log["val_rmse_deg"].append(_rmse_deg(val_pred, y_val))
        # 只保存验证集最优的那一轮：末尾几轮可能已经过拟合，最后一代不一定最好
        if val_mse < best_mse:
            best_mse, best_epoch = val_mse, ep
            best_params = [p.copy() for p in model._params()]
        if verbose and (ep % 10 == 0 or ep == epochs-1):
            print(f"  epoch {ep:3d}/{epochs} | lr={cur_lr:.4f} | train_mse={np.mean(losses):.6f} "
                  f"| val_mse={val_mse:.6f} | val_rmse={log['val_rmse_deg'][-1]:.2f}°")
    if best_params is not None:
        for p, b in zip(model._params(), best_params):
            p[...] = b
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    model.save(out_path)
    log["final_val_mse"] = log["val_mse"][-1]
    log["best_val_mse"] = best_mse
    log["best_epoch"] = best_epoch
    log["best_val_rmse_deg"] = log["val_rmse_deg"][best_epoch] if best_epoch >= 0 else float("inf")
    log["weight_path"] = out_path
    if verbose:
        print(f"[train] saved {out_path} (best epoch {best_epoch}/{epochs}), "
              f"best val_mse={best_mse:.6f} ({log['best_val_rmse_deg']:.2f}°)")
    return log
