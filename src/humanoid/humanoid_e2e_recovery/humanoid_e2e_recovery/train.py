"""train.py —— 训练端到端 MLP（图像→力矩增量回归）。"""
from __future__ import annotations
import os
import numpy as np
from .e2e_nn import E2EMLP
from .dataset import TEMPORAL_LAG_SEC, X_STD_FLOOR, make_pairs

DATASET_HINT = ("数据集文件不存在: {p}\n"
                "请先采集数据：  python3 main.py --collect --episodes 60")


def train(dataset_path, out_weight_path, in_dim=None, h1=128, h2=64, out_dim=15,
          epochs=200, batch=64, lr=0.005, val_ratio=0.2, seed=0, verbose=True,
          lag_sec=TEMPORAL_LAG_SEC):
    if not os.path.exists(dataset_path):
        raise FileNotFoundError(DATASET_HINT.format(p=dataset_path))
    d = np.load(dataset_path)
    X, Y = d["X"].astype(np.float64), d["Y"].astype(np.float64)
    if len(Y) < 100:
        raise ValueError(f"数据集样本过少（{len(Y)} 条，至少 100 条），"
                         f"请重新执行 --collect 采集更多数据")

    # —— 双帧时序输入 ——
    # 单帧里没有速度信息，闭环阻尼不足（实测上限 3.53 s，见 diag/09）。
    # 把 t 与 t-0.102 s 两帧并排喂进去，网络就能自己差分出"往哪边倒"。
    # ep 字段用于保证配对不跨回合；旧数据集没有 ep 时退回单帧。
    if lag_sec and "ep" in d and len(d["ep"]) == len(X):
        sample_dt = float(d["sample_dt"]) if "sample_dt" in d else 0.006
        X, Y, lag = make_pairs(X, Y, d["ep"], lag_sec, sample_dt)
        print(f"[train] 双帧输入: lag={lag} 个采样 ≈ {lag_sec * 1000:.0f} ms | X={X.shape}")
    else:
        lag = 0
        print(f"[train] 单帧输入: X={X.shape}")

    in_dim = in_dim or X.shape[1]

    # —— 输入标准化 ——
    # 16×16 池化灰度图的动态范围很窄（实测 0.37~0.91，逐维 std 均值仅 0.019）。
    # 若直接喂原始值：输入全为非负、均值约 0.58，与权重内积带上 -0.58*colsum(W1)
    # 的系统性偏置，第一层 128 个 ReLU 会被整片推到负半轴而全部死亡，
    # 网络退化成 y = W3·ReLU(b2) + b3，输出与图像完全无关。
    # 双帧输入是新拼出来的，数据集里存的 256 维 X_mean/X_std 不能再用，
    # 必须按配对后的样本现算（否则维度直接对不上）。
    X_mean = (d["X_mean"].astype(np.float64) if ("X_mean" in d and not lag)
              else X.mean(axis=0))
    X_std = (d["X_std"].astype(np.float64) if ("X_std" in d and not lag)
             else np.maximum(X.std(axis=0), X_STD_FLOOR))
    Xn = (X - X_mean) / X_std

    # —— 输出标准化 ——
    # 尺度一律从当前数据集现算：旧权重里那份 Y_scale 属于更早的数据集版本
    # （存 13.79 / 0.327，实际数据集对应维度是 0.075 / 0.009），按它归一化会把
    # 目标放大到 1e5 量级；且恒定维度的尺度必须取 1.0，不能取 1e-6。
    Y_mean = (d["Y_mean"].astype(np.float64) if "Y_mean" in d
              else Y.mean(axis=0))
    y_std = Y.std(axis=0)
    Y_scale = np.where(y_std > 1e-6, y_std, 1.0)
    Yn = (Y - Y_mean) / Y_scale

    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(Yn))
    Xn, Yn = Xn[idx], Yn[idx]
    nv = max(1, int(len(Yn) * val_ratio))
    Xv, Yv = Xn[:nv], Yn[:nv]
    Xt, Yt = Xn[nv:], Yn[nv:]
    model = E2EMLP(in_dim, h1, h2, out_dim, seed=seed)
    val_mse = float("nan")
    for ep in range(epochs):
        perm = rng.permutation(len(Yt))
        Xt, Yt = Xt[perm], Yt[perm]
        losses = []
        cur_lr = lr * (0.5 if ep > epochs * 0.6 else 1.0)
        for i in range(0, len(Yt), batch):
            losses.append(model.train_step(Xt[i:i + batch], Yt[i:i + batch], lr=cur_lr))
        pred_v = model.predict(Xv)
        val_mse = float(np.mean((pred_v - Yv) ** 2))
        if verbose and (ep % 20 == 0 or ep == epochs - 1):
            print(f"  epoch {ep:3d}/{epochs} | train_loss={np.mean(losses):.6f} "
                  f"| val_mse(norm)={val_mse:.6f}")
    os.makedirs(os.path.dirname(out_weight_path) or ".", exist_ok=True)
    model.save(out_weight_path, extra={"Y_scale": Y_scale, "Y_mean": Y_mean,
                                       "X_mean": X_mean, "X_std": X_std,
                                       "X_LAG_SEC": np.float64(lag_sec if lag else 0.0),
                                       "X_LAG": np.int64(lag)})
    if verbose:
        mse_phys = float(np.mean(((model.predict(Xv) - Yv) * Y_scale) ** 2))
        print(f"[train] 权重: {out_weight_path} | val_mse(norm)={val_mse:.5f} "
              f"| val_mse(物理, N·m^2)={mse_phys:.4f}")
    return {"val_mse": val_mse, "weight_path": out_weight_path,
            "y_scale": Y_scale, "x_mean": X_mean, "x_std": X_std}
