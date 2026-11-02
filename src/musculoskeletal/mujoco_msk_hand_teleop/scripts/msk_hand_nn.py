#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""神经网络逆模型：食指指尖目标位置 → 11 维肌肉激活。

流程：
  1. 采集数据：随机采样 6 维相关肌肉激活（腕屈/伸、腕左/右摆、食指屈/伸），
     在 free 场景中让手达到静平衡，记录 (指尖位置 x, 激活 a) 样本；
  2. 训练 MLP（3→64→64→11，ReLU 隐层 + Sigmoid 输出，Adam + MSE）；
  3. 训练岭回归线性基线做对比；
  4. 测试集评估：NN / 线性基线预测的激活下发到仿真，静平衡后测量到达误差；
  5. 保存权重 assets/nn_weights.npz、指标 assets/nn_metrics.json，
     训练图表写 docs/img/msk_hand_nn_training.png（安装空间回退 assets/）。

用法：python3 msk_hand_nn.py [--n-train 1500] [--epochs 300]
"""
import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from msk_hand_core import MskHandSim, MUSCLE_NAMES, N_MUSCLE  # noqa: E402

from msk_hand_core import PKG_DIR  # noqa: E402
ASSET_DIR = os.path.join(PKG_DIR, "assets")
# 训练图表写入 docs/img/（与 mkdocs 引用一致）；安装空间无 docs/ 时回退 assets/
DOCS_IMG_DIR = os.path.normpath(
    os.path.join(PKG_DIR, os.pardir, os.pardir, os.pardir, "docs", "img"))
CHART_DIR = DOCS_IMG_DIR if os.path.isdir(DOCS_IMG_DIR) else ASSET_DIR
CHART_PREFIX = "msk_hand_" if CHART_DIR == DOCS_IMG_DIR else ""

# 影响食指指尖位姿的肌肉子集（其余肌肉置 0，保证逆映射可学习）
CTRL_MUSCLES = ["m_wrist_flex", "m_wrist_ext", "m_wrist_yaw_l", "m_wrist_yaw_r",
                "m_index_flex", "m_index_ext"]
CTRL_IDX = [MUSCLE_NAMES.index(m) for m in CTRL_MUSCLES]

RNG = np.random.RandomState(0)


# ------------------------------ 数据采集 ------------------------------ #
def collect(n_samples, settle=120, seed=0):
    """随机激活采样，返回 X=(N,3) 指尖位置, Y=(N,11) 激活。"""
    rng = np.random.RandomState(seed)
    sim = MskHandSim("free")
    X, Y = [], []
    for i in range(n_samples):
        a = np.zeros(N_MUSCLE)
        a[CTRL_IDX] = rng.uniform(0.0, 1.0, size=len(CTRL_IDX))
        sim.reset()
        for _ in range(settle):
            sim.step(a)
        X.append(sim.index_tip().copy())
        Y.append(a.copy())
        if (i + 1) % 200 == 0:
            print("  采集 %d/%d" % (i + 1, n_samples), flush=True)
    return np.array(X), np.array(Y)


# ------------------------------ MLP 模型 ------------------------------ #
class MLP:
    """3→64→64→11，ReLU 隐层，Sigmoid 输出，Adam 优化，MSE 损失。"""

    def __init__(self, dims=(3, 64, 64, 11), seed=0):
        rng = np.random.RandomState(seed)
        self.W, self.b = [], []
        for f_in, f_out in zip(dims[:-1], dims[1:]):
            scale = np.sqrt(2.0 / f_in)           # He 初始化
            self.W.append(rng.randn(f_in, f_out) * scale)
            self.b.append(np.zeros(f_out))
        # Adam 状态
        self.mW = [np.zeros_like(w) for w in self.W]
        self.vW = [np.zeros_like(w) for w in self.W]
        self.mb = [np.zeros_like(b) for b in self.b]
        self.vb = [np.zeros_like(b) for b in self.b]
        self.t = 0

    def forward(self, x):
        h1 = np.maximum(0.0, x @ self.W[0] + self.b[0])
        h2 = np.maximum(0.0, h1 @ self.W[1] + self.b[1])
        y = 1.0 / (1.0 + np.exp(-(h2 @ self.W[2] + self.b[2])))  # sigmoid
        cache = (x, h1, h2, y)
        return y, cache

    def loss(self, y_pred, y_true):
        return float(np.mean((y_pred - y_true) ** 2))

    def backward(self, cache, y_true):
        x, h1, h2, y = cache
        n = x.shape[0]
        dy = 2.0 * (y - y_true) / n                 # dMSE/dy
        dh2 = dy * y * (1.0 - y)                    # sigmoid 反传
        gW2 = h2.T @ dh2
        gb2 = dh2.sum(0)
        dh1 = (dh2 @ self.W[2].T) * (h2 > 0)
        gW1 = h1.T @ dh1
        gb1 = dh1.sum(0)
        dx = (dh1 @ self.W[1].T) * (h1 > 0)
        gW0 = x.T @ dx
        gb0 = dx.sum(0)
        return [gW0, gW1, gW2], [gb0, gb1, gb2]

    def adam_step(self, grads_w, grads_b, lr=1e-3, beta1=0.9, beta2=0.999, eps=1e-8):
        self.t += 1
        for i, (gw, gb) in enumerate(zip(grads_w, grads_b)):
            for param, grad, m, v in ((self.W[i], gw, self.mW[i], self.vW[i]),
                                      (self.b[i], gb, self.mb[i], self.vb[i])):
                m *= beta1
                m += (1 - beta1) * grad
                v *= beta2
                v += (1 - beta2) * grad * grad
                m_hat = m / (1 - beta1 ** self.t)
                v_hat = v / (1 - beta2 ** self.t)
                param -= lr * m_hat / (np.sqrt(v_hat) + eps)

    def save(self, path, x_mean, x_std):
        d = {"x_mean": x_mean, "x_std": x_std}
        for i, (w, b) in enumerate(zip(self.W, self.b)):
            d["W%d" % i] = w
            d["b%d" % i] = b
        np.savez(path, **d)

    @classmethod
    def load(cls, path):
        d = np.load(path)
        net = cls()
        net.W = [d["W%d" % i] for i in range(3)]
        net.b = [d["b%d" % i] for i in range(3)]
        return net, d["x_mean"], d["x_std"]


def ridge_baseline(X, Y, lam=1e-3):
    """岭回归闭式解（带偏置），返回预测函数。"""
    Xb = np.hstack([X, np.ones((len(X), 1))])
    A = Xb.T @ Xb + lam * np.eye(Xb.shape[1])
    coef = np.linalg.solve(A, Xb.T @ Y)

    def predict(x):
        xb = np.hstack([x, np.ones((x.shape[0], 1))])
        return np.clip(xb @ coef, 0.0, 1.0)
    return predict


# ------------------------------ 评估 ---------------------------------- #
def eval_reach(predict_fn, X_in, X_tgt, settle=120):
    """把预测激活下发仿真，静平衡后测指尖到达误差（米）。

    X_in    : 预测函数的输入（已按预测函数要求预处理）
    X_tgt   : 原始米制目标位置（用于计算误差）
    """
    sim = MskHandSim("free")
    errs = []
    for i in range(len(X_in)):
        a = np.clip(predict_fn(X_in[i:i + 1])[0], 0.0, 1.0)
        sim.reset()
        for _ in range(settle):
            sim.step(a)
        errs.append(np.linalg.norm(sim.index_tip() - X_tgt[i]))
    return np.array(errs)


# ------------------------------ 主流程 -------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-train", type=int, default=1500)
    ap.add_argument("--n-test", type=int, default=80)
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--batch", type=int, default=128)
    ap.add_argument("--lr", type=float, default=2e-3)
    args = ap.parse_args()

    os.makedirs(ASSET_DIR, exist_ok=True)

    print("[1/4] 采集数据 ...", flush=True)
    X_tr, Y_tr = collect(args.n_train, seed=1)
    X_te, Y_te = collect(args.n_test, seed=2)
    x_mean, x_std = X_tr.mean(0), X_tr.std(0) + 1e-8
    Xn_tr = (X_tr - x_mean) / x_std
    Xn_te = (X_te - x_mean) / x_std

    print("[2/4] 训练 MLP ...", flush=True)
    net = MLP()
    losses = []
    n = len(Xn_tr)
    for ep in range(args.epochs):
        perm = RNG.permutation(n)
        ep_loss = 0.0
        for s in range(0, n, args.batch):
            idx = perm[s:s + args.batch]
            y, cache = net.forward(Xn_tr[idx])
            ep_loss += net.loss(y, Y_tr[idx]) * len(idx)
            gw, gb = net.backward(cache, Y_tr[idx])
            net.adam_step(gw, gb, lr=args.lr)
        losses.append(ep_loss / n)
        if (ep + 1) % 50 == 0:
            print("  epoch %d loss=%.5f" % (ep + 1, losses[-1]), flush=True)

    print("[3/4] 线性基线 ...", flush=True)
    lin = ridge_baseline(Xn_tr, Y_tr)

    print("[4/4] 测试集评估（仿真到达误差）...", flush=True)
    err_nn = eval_reach(lambda x: net.forward((x - x_mean) / x_std)[0],
                        X_te, X_te)
    err_lin = eval_reach(lin, Xn_te, X_te)

    metrics = {
        "n_train": args.n_train, "n_test": args.n_test,
        "epochs": args.epochs,
        "final_train_mse": losses[-1],
        "nn_error_m": {"mean": float(err_nn.mean()),
                       "p90": float(np.percentile(err_nn, 90)),
                       "max": float(err_nn.max())},
        "linear_error_m": {"mean": float(err_lin.mean()),
                           "p90": float(np.percentile(err_lin, 90)),
                           "max": float(err_lin.max())},
    }
    with open(os.path.join(ASSET_DIR, "nn_metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    net.save(os.path.join(ASSET_DIR, "nn_weights.npz"), x_mean, x_std)

    # 图表：损失曲线 + 误差对比
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.4))
    axes[0].plot(losses)
    axes[0].set_xlabel("epoch"); axes[0].set_ylabel("MSE loss")
    axes[0].set_title("MLP training loss")
    labels = ["mean", "p90", "max"]
    nn_v = [err_nn.mean(), np.percentile(err_nn, 90), err_nn.max()]
    lin_v = [err_lin.mean(), np.percentile(err_lin, 90), err_lin.max()]
    w = 0.35; xs = np.arange(3)
    axes[1].bar(xs - w / 2, nn_v, w, label="MLP")
    axes[1].bar(xs + w / 2, lin_v, w, label="Linear")
    axes[1].set_xticks(xs); axes[1].set_xticklabels(labels)
    axes[1].set_ylabel("reach error (m)"); axes[1].set_title("Test reach error")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(CHART_DIR, CHART_PREFIX + "nn_training.png"), dpi=130)

    print("NN  误差: mean=%.1fmm p90=%.1fmm" %
          (err_nn.mean() * 1e3, np.percentile(err_nn, 90) * 1e3))
    print("线性误差: mean=%.1fmm p90=%.1fmm" %
          (err_lin.mean() * 1e3, np.percentile(err_lin, 90) * 1e3))
    print("已保存: nn_weights.npz / nn_metrics.json / nn_training.png")


if __name__ == "__main__":
    main()
