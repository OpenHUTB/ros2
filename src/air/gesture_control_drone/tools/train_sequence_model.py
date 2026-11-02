#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练手势时序模型并导出为纯 numpy 权重（推理端不需要 torch）。

两种后端
--------
* ``--backend ridge``（默认，**不需要 torch**）：
  把随机初始化的 LSTM 当作"储层"，只训练最后的线性分类头（岭回归）。
  训练是闭式解，秒级完成，适合冷启动和回归测试。
* ``--backend torch``：用 PyTorch 端到端训练（反向传播穿过 LSTM），
  准确率更高，适合真实数据。装不上 torch 时用 ridge 即可。

两种后端导出的是**同一种 npz 格式**，推理端 :class:`sequence_model.LSTMModel` 通用。

示例
----
    # 合成数据 + 岭回归，10 秒内跑完
    python3 tools/train_sequence_model.py --samples 80 --out models/gesture_seq.npz

    # 真实数据（先用 tools/record_sequences.py 采集） + torch 训练
    python3 tools/train_sequence_model.py --data data/seq.npz --backend torch --epochs 30
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(os.path.dirname(_HERE), "scripts")
for p in (_SCRIPTS, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import sequence_model as sm            # noqa: E402


def confusion_matrix(y_true, y_pred, n_classes):
    cm = np.zeros((n_classes, n_classes), dtype=np.int64)
    for t, p in zip(y_true, y_pred):
        cm[int(t), int(p)] += 1
    return cm


def print_report(y_true, y_pred, classes):
    n = len(classes)
    cm = confusion_matrix(y_true, y_pred, n)
    acc = float(np.trace(cm)) / max(1, cm.sum())
    print("\n准确率: %.4f  (n=%d)" % (acc, cm.sum()))
    print("\n混淆矩阵（行=真实，列=预测）")
    header = "".join("%8s" % c[:7] for c in classes)
    print("%12s%s" % ("", header))
    for i, c in enumerate(classes):
        row = "".join("%8d" % v for v in cm[i])
        print("%12s%s" % (c[:11], row))
    per_class = []
    for i, c in enumerate(classes):
        total = cm[i].sum()
        per_class.append((c, float(cm[i, i]) / total if total else 0.0, int(total)))
    print("\n每类召回率")
    for c, r, total in per_class:
        print("  %-12s %.3f  (n=%d)" % (c, r, total))
    return acc, cm



def fit_temperature(model, xs, masks, ys, grid=None):
    """在验证集上最小化 NLL 拟合 softmax 温度（不改变 argmax）。"""
    logits = np.stack([model.logits(x, m) for x, m in zip(xs, masks)])
    ys = np.asarray(ys, dtype=np.int64)
    grid = grid if grid is not None else np.exp(np.linspace(np.log(0.02),
                                                            np.log(3.0), 60))
    # 标签平滑：合成数据常常"完美可分"，不加平滑时最优温度会退化到网格下界，
    # softmax 被压成 0/1 的硬判决，反而更容易在过渡帧上误触发。
    eps = 0.05
    n_cls = logits.shape[1]
    target = np.full_like(logits, eps / max(1, n_cls - 1), dtype=np.float64)
    target[np.arange(len(ys)), ys] = 1.0 - eps

    best_t, best_nll = 1.0, float("inf")
    for T in grid:
        z = logits / float(T)
        z = z - z.max(axis=1, keepdims=True)
        e = np.exp(z)
        p = e / e.sum(axis=1, keepdims=True)
        nll = float(-np.mean(np.sum(target * np.log(p + 1e-9), axis=1)))
        if nll < best_nll:
            best_t, best_nll = float(T), nll
    return best_t, best_nll


def train_ridge(x_tr, m_tr, y_tr, x_va, m_va, y_va, hidden=64, layers=1,
                seed=0, ridge=1.0):
    """储层 + 岭回归：只学最后的线性头。"""
    model = sm.LSTMModel.random(hidden=hidden, layers=layers, seed=seed)
    t0 = time.time()
    h_tr = np.stack([model.forward_hidden(x, m) for x, m in zip(x_tr, m_tr)])
    h_va = np.stack([model.forward_hidden(x, m) for x, m in zip(x_va, m_va)])
    print("  前向完成（%d 训练 + %d 验证），用时 %.2fs"
          % (len(h_tr), len(h_va), time.time() - t0))

    mu = h_tr.mean(axis=0, keepdims=True)
    sd = h_tr.std(axis=0, keepdims=True) + 1e-6
    hs = (h_tr - mu) / sd

    n_cls = len(model.classes)
    Y = np.zeros((len(y_tr), n_cls), dtype=np.float32)
    Y[np.arange(len(y_tr)), y_tr] = 1.0

    A = hs.T @ hs + float(ridge) * np.eye(hs.shape[1])
    W = np.linalg.solve(A, hs.T @ Y)                 # (hidden, n_cls)
    # 把标准化折回原始隐状态：logits = H @ (W/sd) - mu/sd @ W
    W_raw = (W / sd.T).T.astype(np.float32)          # (n_cls, hidden)
    b_raw = (-(mu / sd) @ W).reshape(-1).astype(np.float32)

    model.params["W_out"] = W_raw
    model.params["b_out"] = b_raw

    # 温度校准：让"置信度"这个数变得可信（阈值才调得动）
    best_t, nll = fit_temperature(model, x_va, m_va, y_va)
    model.temperature = best_t
    print("  温度校准: T=%.3f（验证集 NLL=%.3f）" % (best_t, nll))
    return model


def train_torch(x_tr, m_tr, y_tr, x_va, m_va, y_va, hidden=64, layers=1,
                epochs=30, lr=3e-3, seed=0):
    """PyTorch 端到端训练；缺帧帧不参与状态更新（与 numpy 前向一致）。"""
    try:
        import torch
        import torch.nn as nn
    except ImportError:
        raise SystemExit("没装 torch；请用 --backend ridge，或 pip install torch")

    torch.manual_seed(seed)
    n_cls = len(sm.SEQUENCE_CLASSES)

    class Net(nn.Module):
        def __init__(self):
            super().__init__()
            self.lstm = nn.LSTM(input_size=sm.FEATURE_DIM, hidden_size=hidden,
                                num_layers=layers, batch_first=True)
            self.fc = nn.Linear(hidden, n_cls)

        def forward(self, x, mask):
            # 逐时间步喂，缺帧（mask=0）保持状态不变 —— 与 numpy 前向同一语义
            B = x.size(0)
            h = torch.zeros(self.lstm.num_layers, B, hidden, device=x.device)
            c = torch.zeros_like(h)
            for t in range(x.size(1)):
                inp = x[:, t, :].unsqueeze(1)
                out, (hn, cn) = self.lstm(inp, (h, c))
                valid = (mask[:, t] > 0)
                h = torch.where(valid.view(1, B, 1), hn, h)
                c = torch.where(valid.view(1, B, 1), cn, c)
            return self.fc(h[-1])

    net = Net()
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    lossf = nn.CrossEntropyLoss()

    xt = torch.from_numpy(x_tr).float()
    mt = torch.from_numpy(m_tr).float()
    yt = torch.from_numpy(y_tr).long()
    xv = torch.from_numpy(x_va).float()
    mv = torch.from_numpy(m_va).float()
    yv = torch.from_numpy(y_va).long()

    for ep in range(1, int(epochs) + 1):
        net.train()
        opt.zero_grad()
        logits = net(xt, mt)
        loss = lossf(logits, yt)
        loss.backward()
        opt.step()
        if ep % max(1, int(epochs) // 5) == 0 or ep == 1:
            net.eval()
            with torch.no_grad():
                pred = net(xv, mv).argmax(dim=1)
                acc = float((pred == yv).float().mean())
            print("  epoch %3d  loss=%.4f  验证准确率=%.4f" % (ep, float(loss), acc))

    params = {}
    for l in range(layers):
        layer = net.lstm
        params["W_ih_%d" % l] = getattr(layer, "weight_ih_l%d" % l).detach().numpy()
        params["W_hh_%d" % l] = getattr(layer, "weight_hh_l%d" % l).detach().numpy()
        params["b_ih_%d" % l] = getattr(layer, "bias_ih_l%d" % l).detach().numpy()
        params["b_hh_%d" % l] = getattr(layer, "bias_hh_l%d" % l).detach().numpy()
    params["W_out"] = net.fc.weight.detach().numpy()
    params["b_out"] = net.fc.bias.detach().numpy()
    params["__layers__"] = np.array([layers], dtype=np.int32)
    params["__hidden__"] = np.array([hidden], dtype=np.int32)
    params["__input_dim__"] = np.array([sm.FEATURE_DIM], dtype=np.int32)
    model = sm.LSTMModel(params)
    best_t, nll = fit_temperature(model, x_va, m_va, y_va)
    model.temperature = best_t
    print("  温度校准: T=%.3f（验证集 NLL=%.3f）" % (best_t, nll))
    return model


def main(argv=None):
    ap = argparse.ArgumentParser(description="训练手势时序模型（ridge 或 torch）")
    ap.add_argument("--backend", choices=("ridge", "torch"), default="ridge")
    ap.add_argument("--samples", type=int, default=80, help="每类合成样本数")
    ap.add_argument("--length", type=int, default=sm.SEQ_LEN)
    ap.add_argument("--hidden", type=int, default=64)
    ap.add_argument("--layers", type=int, default=1)
    ap.add_argument("--epochs", type=int, default=30, help="torch 后端用")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--val-ratio", type=float, default=0.25)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(_HERE),
                                                  "models", "gesture_seq.npz"))
    ap.add_argument("--data", default=None, help="真实数据 npz（x,y 数组）")
    args = ap.parse_args(argv)

    if args.data:
        with np.load(args.data, allow_pickle=False) as d:
            seqs = np.asarray(d["x"], dtype=np.float32)
            labels = np.asarray(d["y"], dtype=np.int64)
        print("载入真实数据:", seqs.shape)
        from synthetic_sequences import mask_of
        masks = mask_of(seqs)
    else:
        import synthetic_sequences as gen
        seqs, labels, _ = gen.make_dataset(samples_per_class=args.samples,
                                           length=args.length, seed=args.seed)
        masks = gen.mask_of(seqs)
        print("合成数据:", seqs.shape, "（仅用于跑通流程，真实指标请用真实数据）")

    import synthetic_sequences as gen
    feats = gen.to_features(seqs)

    n = len(labels)
    rng = np.random.RandomState(args.seed)
    idx = rng.permutation(n)
    n_val = max(1, int(round(n * args.val_ratio)))
    va, tr = idx[:n_val], idx[n_val:]

    print("训练 %d / 验证 %d，后端=%s" % (len(tr), len(va), args.backend))
    if args.backend == "ridge":
        model = train_ridge(feats[tr], masks[tr], labels[tr],
                           feats[va], masks[va], labels[va],
                           hidden=args.hidden, layers=args.layers,
                           seed=args.seed)
    else:
        model = train_torch(feats[tr], masks[tr], labels[tr],
                            feats[va], masks[va], labels[va],
                            hidden=args.hidden, layers=args.layers,
                            epochs=args.epochs, seed=args.seed)

    y_pred = np.array([np.argmax(model.predict_proba(x, m))
                       for x, m in zip(feats[va], masks[va])])
    acc, _ = print_report(labels[va], y_pred, list(model.classes))

    path = model.save(args.out)
    size_kb = os.path.getsize(path) / 1024.0
    print("\n模型已保存: %s  (%.1f KB, hidden=%d, layers=%d)"
          % (path, size_kb, model.hidden, model.layers))
    print("推理端只需 numpy：sequence_model.LSTMModel.load('%s')" % path)
    return 0 if acc >= 0.5 else 1


if __name__ == "__main__":
    sys.exit(main())
