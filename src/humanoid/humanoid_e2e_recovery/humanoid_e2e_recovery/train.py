"""train.py —— 训练端到端 MLP（图像→力矩增量回归）。"""
from __future__ import annotations
import os
import numpy as np
from .e2e_nn import E2EMLP

DATASET_HINT = ("数据集文件不存在: {p}\n"
                "请先采集数据：  python3 main.py --collect --episodes 60")


def train(dataset_path, out_weight_path, in_dim=256, h1=128, h2=64, out_dim=15,
          epochs=80, batch=64, lr=0.005, val_ratio=0.2, seed=0, verbose=True):
    if not os.path.exists(dataset_path):
        raise FileNotFoundError(DATASET_HINT.format(p=dataset_path))
    d = np.load(dataset_path)
    X, Y = d["X"].astype(np.float64), d["Y"].astype(np.float64)
    if len(Y) < 100:
        raise ValueError(f"数据集样本过少（{len(Y)} 条，至少 100 条），"
                         f"请重新执行 --collect 采集更多数据")
    # 逐维标准化：力矩各维量级差异极大，不归一化只能学到大的一维
    scale = d["Y_scale"].astype(np.float64) if "Y_scale" in d else np.maximum(Y.std(0), 1e-6)
    Yn = Y / scale

    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(Yn))
    X, Yn = X[idx], Yn[idx]
    nv = max(1, int(len(Yn) * val_ratio))
    Xv, Yv = X[:nv], Yn[:nv]
    Xt, Yt = X[nv:], Yn[nv:]
    model = E2EMLP(in_dim, h1, h2, out_dim, seed=seed)
    val_mse = float("nan")
    for ep in range(epochs):
        perm = rng.permutation(len(Yt))
        Xt, Yt = Xt[perm], Yt[perm]
        losses = []
        for i in range(0, len(Yt), batch):
            losses.append(model.train_step(Xt[i:i + batch], Yt[i:i + batch], lr=lr))
        pred_v = model.predict(Xv)
        val_mse = float(np.mean((pred_v - Yv) ** 2))
        if verbose and (ep % 10 == 0 or ep == epochs - 1):
            print(f"  epoch {ep:3d}/{epochs} | train_loss={np.mean(losses):.5f} "
                  f"| val_mse(norm)={val_mse:.5f}")
    os.makedirs(os.path.dirname(out_weight_path) or ".", exist_ok=True)
    model.save(out_weight_path, extra={"Y_scale": scale})
    if verbose:
        # 还原成物理量纲的 MSE 更有意义
        mse_phys = float(np.mean(((model.predict(Xv) - Yv) * scale) ** 2))
        print(f"[train] 权重: {out_weight_path} | val_mse(norm)={val_mse:.5f} "
              f"| val_mse(物理, N·m^2)={mse_phys:.4f}")
    return {"val_mse": val_mse, "weight_path": out_weight_path, "y_scale": scale}
