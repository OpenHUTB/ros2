"""训练四个神经网络并输出指标。

1. 感知网络：9 线滑窗 -> 中心线"去噪后"距离（MLP）
2. 规划网络：局部 11×11 占据栅格 + 目标 -> 局部前进方向 (cosφ, sinφ)（MLP）
3. 控制网络：目标距离/航向误差/三向净空/上一速度 -> (v, ω)（MLP）
4. 端到端网络：24×32 深度图 -> (v, ω)（CNN）
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from aerial_mujoco_slam_nn_nav import nn  # noqa: E402


def sector_min(scan: np.ndarray) -> np.ndarray:
    n = scan.shape[-1]
    front = scan[:, n // 2 - 15: n // 2 + 15].min(axis=1)
    left = scan[:, n // 2 + 15: n // 2 + 50].min(axis=1)
    right = scan[:, n // 2 - 50: n // 2 - 15].min(axis=1)
    return np.stack([front, left, right], axis=1)


def main():
    data = np.load("data/dataset.npz")
    scan, clean, patch = data["scan"], data["scan_clean"], data["patch"]
    goal, cmd, depth, direction = data["goal"], data["cmd"], data["depth"], data["dir"]
    rng = np.random.default_rng(0)
    n = len(scan)
    metrics = {"frames": int(n)}

    # ---------------- 1. 感知去噪 ----------------
    w = 4
    win = np.stack([np.roll(scan, k, axis=1) for k in range(-w, w + 1)], axis=-1).reshape(-1, 2 * w + 1)
    tgt = clean.reshape(-1, 1)
    idx = rng.permutation(len(win))[:120000]
    X, Y = win[idx] / 8.0, tgt[idx] / 8.0
    cut = int(0.85 * len(X))
    p = nn.init_mlp([2 * w + 1, 32, 32, 1], rng)
    hist = nn.train_mlp(p, X[:cut], Y[:cut], epochs=25, batch=128, lr=2e-3,
                        val=(X[cut:], Y[cut:]), verbose=True)
    nn.save_model("models/perception.npz", p)
    raw_rmse = float(np.sqrt(np.mean((win[idx][cut:] / 8.0 - Y[cut:]) ** 2)) * 8.0)
    pred = nn.mlp_forward(p, X[cut:]) * 8.0
    net_rmse = float(np.sqrt(np.mean((pred - Y[cut:] * 8.0) ** 2)))
    metrics["perception"] = {"raw_rmse_m": round(raw_rmse, 4), "nn_rmse_m": round(net_rmse, 4),
                             "loss": round(hist[-1]["loss"], 6)}

    # ---------------- 2. 神经局部规划 ----------------
    moving = np.abs(cmd[:, 0]) > 0.01
    P = patch[moving].reshape(-1, 121)
    G = goal[moving][:, :2]
    gl = np.linalg.norm(G, axis=1, keepdims=True) + 1e-6
    Xp = np.concatenate([P, G, gl], axis=1).astype(np.float32)
    Yp = direction[moving]
    idx = rng.permutation(len(Xp))
    Xp, Yp = Xp[idx], Yp[idx]
    cut = int(0.85 * len(Xp))
    pm = nn.init_mlp([Xp.shape[1], 64, 64, 2], rng)
    hist = nn.train_mlp(pm, Xp[:cut], Yp[:cut], epochs=60, batch=64, lr=1e-3,
                        val=(Xp[cut:], Yp[cut:]), verbose=True)
    nn.save_model("models/planner.npz", pm)
    pv = nn.mlp_forward(pm, Xp[cut:])
    ang_err = np.degrees(np.abs(np.arctan2(pv[:, 1], pv[:, 0]) -
                                np.arctan2(Yp[cut:, 1], Yp[cut:, 0])))
    metrics["planner"] = {"val_mae_deg": round(float(np.mean(ang_err)), 2),
                          "val_acc_45deg": round(float(np.mean(ang_err < 45)), 3),
                          "loss": round(hist[-1]["loss"], 6), "samples": int(len(Xp))}

    # ---------------- 3. 神经控制器 ----------------
    sec = sector_min(scan)
    feats = np.concatenate([goal[:, 2:3], np.arcsin(np.clip(goal[:, 1:2] / (goal[:, 2:3] + 1e-6), -1, 1)),
                            sec, np.zeros((n, 1))], axis=1).astype(np.float32)
    feats[:, -1] = np.concatenate([[0.0], cmd[:-1, 0]])
    Xc, Yc = feats, cmd.astype(np.float32)
    cut = int(0.85 * len(Xc))
    pc = nn.init_mlp([Xc.shape[1], 64, 64, 3], rng)
    hist = nn.train_mlp(pc, Xc[:cut], Yc[:cut], epochs=80, batch=64, lr=1e-3,
                        val=(Xc[cut:], Yc[cut:]), verbose=True)
    nn.save_model("models/controller.npz", pc)
    pv = nn.mlp_forward(pc, Xc[cut:])
    metrics["controller"] = {
        "val_mae_vx": round(float(np.mean(np.abs(pv[:, 0] - Yc[cut:, 0]))), 4),
        "val_mae_vy": round(float(np.mean(np.abs(pv[:, 1] - Yc[cut:, 1]))), 4),
        "val_mae_w": round(float(np.mean(np.abs(pv[:, 2] - Yc[cut:, 2]))), 4),
        "loss": round(hist[-1]["loss"], 6)}

    # ---------------- 4. 端到端 CNN ----------------
    # 端到端只学"走廊直飞"这一段（episode 2 / 5），输入深度图、输出机体系速度
    scene_id = data["scene"]
    mask = np.isin(scene_id, [2.0, 5.0])
    Xe = (depth[mask] / 8.0).astype(np.float32)
    Ye = cmd[mask].astype(np.float32)
    cut = int(0.85 * len(Xe))
    pe = nn.init_cnn(24, 32, n_filters=8, k=5, stride=2, hidden=32, out_dim=3, rng=rng)
    hist = nn.train_cnn(pe, Xe[:cut], Ye[:cut], epochs=40, batch=32, lr=1e-3, verbose=True)
    nn.save_model("models/e2e.npz", pe)
    pv = nn.cnn_forward(pe, Xe[cut:])
    metrics["e2e"] = {
        "val_mae_vx": round(float(np.mean(np.abs(pv[:, 0] - Ye[cut:, 0]))), 4),
        "val_mae_vy": round(float(np.mean(np.abs(pv[:, 1] - Ye[cut:, 1]))), 4),
        "val_mae_w": round(float(np.mean(np.abs(pv[:, 2] - Ye[cut:, 2]))), 4),
        "samples": int(len(Xe)),
        "loss": round(hist[-1]["loss"], 6)}

    os.makedirs("results", exist_ok=True)
    with open("results/train_metrics.json", "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
