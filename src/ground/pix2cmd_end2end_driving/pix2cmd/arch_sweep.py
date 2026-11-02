"""Architecture sweep: the same end-to-end task with different network structures.

课程要求中包含"同样的任务、不同的模型结构对比"。本文件在同一份合成数据集、
同样的超参数、同一个闭环评价流程下，比较四种结构：

    mlp        无卷积对照：把 48x48 图像展平成 2304 维，两层全连接
    cnn_tiny   Conv(4)  -> Conv(8)  -> FC(16)
    cnn_base   Conv(8)  -> Conv(16) -> FC(32)   （main.py --demo 用的默认结构）
    cnn_wide   Conv(12) -> Conv(24) -> FC(64)

所有网络都只用 numpy 实现前向与反向传播，接口一致（forward/backward/step/
predict/config），因此可以放进同一个训练循环与同一个闭环控制器里直接比较。
"""

from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import numpy as np

from .cnn import TinyCNN
from .dataset import IMG, make_dataset
from .train import closed_loop_drive

# 结构名 -> (c1, c2, hidden)。mlp 走单独的类。
VARIANTS = {
    "cnn_tiny": (4, 8, 16),
    "cnn_base": (8, 16, 32),
    "cnn_wide": (12, 24, 64),
}


class MLPBaseline:
    """无卷积对照：flatten(48x48) -> FC(hidden) -> ReLU -> FC(2)。

    存在的意义是把"卷积"这一项单独拎出来做消融：输入、标签、优化器、
    训练轮数都与 CNN 完全相同，唯一的差别是没有局部感受野与权值共享。
    """

    def __init__(self, in_hw: int = IMG, hidden: int = 64, seed: int = 0):
        rng = np.random.default_rng(seed)
        flat = in_hw * in_hw
        self.shapes = (in_hw, flat)
        self.p = {
            "w1": (rng.normal(0, 0.05, (flat, hidden))).astype(np.float32),
            "b1": np.zeros(hidden, np.float32),
            "w2": (rng.normal(0, 0.05, (hidden, 2))).astype(np.float32),
            "b2": np.zeros(2, np.float32),
        }
        self.m = {k: np.zeros_like(v) for k, v in self.p.items()}
        self.v = {k: np.zeros_like(v) for k, v in self.p.items()}
        self.t = 0

    def forward(self, x: np.ndarray):
        p = self.p
        xf = x.reshape(x.shape[0], -1).astype(np.float32)
        z1 = xf @ p["w1"] + p["b1"]
        a1 = np.maximum(z1, 0.0)
        z2 = a1 @ p["w2"] + p["b2"]
        steer = np.tanh(z2[:, 0])
        throttle = 1.0 / (1.0 + np.exp(-z2[:, 1]))
        return np.stack([steer, throttle], axis=1), (xf, z1, a1, z2, steer, throttle)

    def backward(self, cache, y_true: np.ndarray) -> dict:
        p = self.p
        xf, z1, a1, z2, steer, throttle = cache
        n = xf.shape[0]
        pred = np.stack([steer, throttle], axis=1)
        dpred = 2.0 * (pred - y_true) / n

        dz2 = np.empty_like(z2)
        dz2[:, 0] = dpred[:, 0] * (1.0 - steer ** 2)
        s = throttle
        dz2[:, 1] = dpred[:, 1] * s * (1.0 - s)

        g = {"w2": a1.T @ dz2, "b2": dz2.sum(axis=0)}
        dz1 = (dz2 @ p["w2"].T) * (z1 > 0)
        g["w1"] = xf.T @ dz1
        g["b1"] = dz1.sum(axis=0)
        return g

    def step(self, grads: dict, lr: float = 1e-3) -> None:
        self.t += 1
        b1, b2, eps = 0.9, 0.999, 1e-8
        for key in self.p:
            grad = grads[key]
            self.m[key] = b1 * self.m[key] + (1 - b1) * grad
            self.v[key] = b2 * self.v[key] + (1 - b2) * grad ** 2
            mh = self.m[key] / (1 - b1 ** self.t)
            vh = self.v[key] / (1 - b2 ** self.t)
            self.p[key] -= (lr * mh / (np.sqrt(vh) + eps)).astype(np.float32)

    def predict(self, x: np.ndarray, batch: int = 128) -> np.ndarray:
        out = []
        for i in range(0, x.shape[0], batch):
            pred, _ = self.forward(x[i:i + batch])
            out.append(pred)
        return np.concatenate(out, axis=0)

    def config(self) -> dict:
        return {"in_hw": self.shapes[0], "params": int(sum(v.size for v in self.p.values()))}


def build_net(name: str, seed: int = 0):
    if name == "mlp":
        return MLPBaseline(seed=seed)
    c1, c2, hidden = VARIANTS[name]
    return TinyCNN(c1=c1, c2=c2, hidden=hidden, seed=seed)


def train_net(net, xtr, ytr, xva, yva, epochs: int = 12, batch: int = 32,
              lr: float = 1e-3, seed: int = 0) -> list:
    """与 train.train 相同的训练过程，只是网络由外部传入。"""
    rng = np.random.default_rng(seed)
    history = []
    for _ in range(epochs):
        order = rng.permutation(xtr.shape[0])
        losses = []
        for start in range(0, xtr.shape[0], batch):
            idx = order[start:start + batch]
            pred, cache = net.forward(xtr[idx])
            grads = net.backward(cache, ytr[idx])
            net.step(grads, lr)
            losses.append(float(np.mean((pred - ytr[idx]) ** 2)))
        mae = np.abs(net.predict(xva) - yva).mean(axis=0)
        history.append({"train_mse": float(np.mean(losses)),
                        "val_mae_steering": float(mae[0]),
                        "val_mae_throttle": float(mae[1])})
    return history


def recovery_test(net, steps: int = 80, lateral0: float = 1.0) -> dict:
    """直线回正实验：曲率为 0，车辆从 1.0 m 横向偏差出发，看多快收敛回中心线。

    S 形弯道的跟踪误差被道路曲率本身主导，各网络差别不明显；直线回正只考验
    "看到偏移 -> 打方向"这一条最本质的映射，因此对模型精度更敏感。
    """
    from .dataset import render
    from .train import CURV_DRIFT, DT, STEER_GAIN

    rng = np.random.default_rng(11)
    lateral = lateral0
    signed = []
    for _ in range(steps):
        image = render(0.0, -lateral, rng, noise=0.0)
        steer = float(net.predict(image[None, :, :, None])[0][0])
        lateral += (CURV_DRIFT * 0.0 - STEER_GAIN * steer) * DT
        signed.append(-lateral)          # 误差 = 道路中心(0) - 车辆横向位置

    signed = np.asarray(signed, dtype=np.float32)
    errors = np.abs(signed)
    settled = np.nonzero(errors < 0.1)[0]
    settle_step = int(settled[0]) if settled.size else -1
    overshoot = float(max(0.0, float(signed.max())))   # 冲过中心线到另一侧的最大幅度
    return {
        "settle_steps": settle_step,
        "settle_seconds": round(settle_step * DT, 1) if settle_step >= 0 else -1.0,
        "final_offset_m": round(float(errors[-1]), 3),
        "overshoot_m": round(overshoot, 3),
    }


def sweep(epochs: int = 12, n_train: int = 1200, n_val: int = 300, batch: int = 32,
          lr: float = 1e-3, seed: int = 0, names=None, log=print) -> list:
    """对每种结构跑一次完整训练 + 闭环评价，返回对比记录。"""
    names = names or ["mlp", *VARIANTS]
    xtr, ytr, _ = make_dataset(n_train, seed=seed)
    xva, yva, _ = make_dataset(n_val, seed=seed + 1)

    records = []
    for name in names:
        log(f"-- 结构 {name}: 训练 {epochs} 轮")
        net = build_net(name, seed=seed)
        t0 = time.perf_counter()
        history = train_net(net, xtr, ytr, xva, yva, epochs=epochs, batch=batch,
                            lr=lr, seed=seed)
        train_s = time.perf_counter() - t0
        _, metrics = closed_loop_drive(net, steps=120)
        recovery = recovery_test(net)
        rec = {
            "arch": name,
            "params": net.config()["params"],
            "train_seconds": round(train_s, 2),
            "val_mae_steering": round(history[-1]["val_mae_steering"], 4),
            "val_mae_throttle": round(history[-1]["val_mae_throttle"], 4),
            "mean_abs_lateral_error_m": metrics["mean_abs_lateral_error_m"],
            "steady_abs_lateral_error_m": metrics["steady_abs_lateral_error_m"],
            "max_abs_lateral_error_m": metrics["max_abs_lateral_error_m"],
            "within_lane_ratio": metrics["within_lane_ratio"],
            "settle_seconds": recovery["settle_seconds"],
            "overshoot_m": recovery["overshoot_m"],
        }
        records.append(rec)
        log("   " + json.dumps(rec, ensure_ascii=False))
    return records


def save_comparison(records, out_dir: str | Path) -> Path:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / "arch_comparison.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    (out / "arch_comparison.json").write_text(
        json.dumps(records, indent=2, ensure_ascii=False), encoding="utf-8")
    return csv_path


def plot_comparison(records, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = [r["arch"] for r in records]
    x = np.arange(len(names))
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 3.8))

    axes[0].bar(x - 0.2, [r["val_mae_steering"] for r in records], width=0.4,
                color="#2f80ed", label="steering")
    axes[0].bar(x + 0.2, [r["val_mae_throttle"] for r in records], width=0.4,
                color="#27ae60", label="throttle")
    axes[0].set_ylabel("validation MAE")
    axes[0].set_title("regression error (lower is better)")
    axes[0].legend(fontsize=8)

    axes[1].bar(x - 0.2, [r["steady_abs_lateral_error_m"] for r in records], width=0.4,
                color="#eb5757", label="steady (t>3s)")
    axes[1].bar(x + 0.2, [r["max_abs_lateral_error_m"] for r in records], width=0.4,
                color="#f2994a", label="max")
    axes[1].set_ylabel("lateral error (m)")
    axes[1].set_title("closed loop tracking error (lower is better)")
    axes[1].legend(fontsize=8)

    axes[2].bar(x, [r["params"] / 1000.0 for r in records], color="#9b51e0")
    for i, r in enumerate(records):
        axes[2].text(i, r["params"] / 1000.0, f"{r['params']/1000:.1f}k",
                     ha="center", va="bottom", fontsize=8)
    axes[2].set_ylabel("parameters (k)")
    axes[2].set_title("model capacity")

    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=15, fontsize=8)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("pix2cmd: same data, same closed-loop test, different architectures",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
