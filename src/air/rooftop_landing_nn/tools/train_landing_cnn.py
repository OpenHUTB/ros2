#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练「学习型屋顶落点选择」小网络（PyTorch，只在开发机跑）。

    python tools/train_landing_cnn.py --epochs 30 --device cuda

产出：
    * models/landing_cnn.npz        —— 权重 + meta（部署端用 numpy 前向，不需要 torch）
    * results/loss.csv              —— 逐 epoch 训练/验证损失
    * results/metrics.json          —— 验证集落点误差等指标
    * docs/img/air/rooftop_landing_nn/training_loss.png

网络结构与 ``scripts/landing_cnn.py`` 的 numpy 实现逐层对应；训练结束后脚本会
**自动做一次 numpy 与 torch 的逐元素对齐检查**，把最大偏差写进 metrics.json，
避免「训练用 torch、部署用 numpy」两边悄悄不一致。

数据来自 ``tools/scene_dataset.py``：512 个理想几何（每 epoch 重新采样稀疏化 /
离群点 / 噪声），真值是理想几何上的最大内切圆圆心。
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(_PKG)))
sys.path.insert(0, os.path.join(_PKG, "scripts"))
sys.path.insert(0, _HERE)

import heightmap as hm_mod          # noqa: E402
import landing_cnn as lcnn          # noqa: E402
import scene_dataset as sd          # noqa: E402

import torch                        # noqa: E402
import torch.nn as nn               # noqa: E402
import torch.nn.functional as F     # noqa: E402

MODEL_DIR = os.path.join(_PKG, "models")
RESULT_DIR = os.path.join(_PKG, "results")
FIG_DIR = os.path.join(_REPO, "docs", "img", "air", "rooftop_landing_nn")


#: 网络层顺序（必须与 scripts/landing_cnn.py 的 numpy 前向逐层一致）
LAYER_NAMES = ("c1", "c2", "c3", "c4", "c5", "c6", "h1")


class LandingCNN(nn.Module):
    """三级下采样 + 三级上采样的小 CNN（结构与 numpy 前向一一对应）。

    感受野：三级 pool 后瓶颈是 8x8（每格 2 m），卷积核一路叠加下来覆盖整个
    16 m ROI，网络才能比较"屋顶中心"和"屋顶边缘"的净空。
    """

    def __init__(self):
        super(LandingCNN, self).__init__()
        self.c1 = nn.Conv2d(3, 16, 3, padding=1)
        self.c2 = nn.Conv2d(16, 32, 3, padding=1)
        self.c3 = nn.Conv2d(32, 48, 3, padding=1)
        self.c4 = nn.Conv2d(48, 48, 3, padding=1)
        self.c5 = nn.Conv2d(48, 24, 3, padding=1)
        self.c6 = nn.Conv2d(24, 16, 3, padding=1)
        self.h1 = nn.Conv2d(16, 1, 1)

    def forward(self, x):
        x = F.max_pool2d(F.relu(self.c1(x)), 2)
        x = F.max_pool2d(F.relu(self.c2(x)), 2)
        x = F.max_pool2d(F.relu(self.c3(x)), 2)
        x = F.interpolate(F.relu(self.c4(x)), scale_factor=2, mode="bilinear", align_corners=False)
        x = F.interpolate(F.relu(self.c5(x)), scale_factor=2, mode="bilinear", align_corners=False)
        x = F.interpolate(F.relu(self.c6(x)), scale_factor=2, mode="bilinear", align_corners=False)
        return self.h1(x)


#: 推理时的平坦度硬约束（`rough` 归一化粗糙度上限）：挡掉贴着墙/边缘下落
ROUGH_MAX = 0.35


def soft_ce_loss(logits, target, valid):
    """「在有效格里选一个格」的软交叉熵。

    为什么不用掩码 MSE：高斯真值只占有效格里的极小一部分（十几个格 / 几百个格），
    MSE 会被大量 0 格支配，网络直接输出全 0 就能把损失压到很低（实测就是这样，
    loss=0.003 但落点误差 9 m）。改成在有效格上做 softmax，直接优化
    「真值格的概率最大」，也就是直接优化 argmax 指标本身。

    logits / target / valid 都是 (B, 1, H, W)。
    """
    b, _, h, w = logits.shape
    neg = torch.full_like(logits, -1e4)
    logits = torch.where(valid > 0.5, logits, neg).view(b, h * w)
    prob = target.view(b, h * w) * (valid.view(b, h * w) > 0.5).float()
    prob = prob / prob.sum(dim=1, keepdim=True).clamp(min=1e-6)
    logp = F.log_softmax(logits, dim=1)
    return -(prob * logp).sum(dim=1).mean()


def export_model(model, path, meta):
    """导出成 npz：键名与 landing_cnn.load_model 约定一致。"""
    w = {}
    for name in LAYER_NAMES:
        layer = getattr(model, name)
        w[name + ".w"] = layer.weight.detach().cpu().numpy().astype(np.float32)
        w[name + ".b"] = layer.bias.detach().cpu().numpy().astype(np.float32)
    w["meta"] = np.array(json.dumps(meta, ensure_ascii=False))
    if not os.path.isdir(os.path.dirname(path)):
        os.makedirs(os.path.dirname(path))
    np.savez_compressed(path, **w)
    return os.path.getsize(path)


def build_batch(scenes, rng, cells, corrupt, batch=32):
    """随机抽 ``batch`` 个场景，各自配一组随机点云退化，组成一个 batch。

    注意必须在这里按 batch 抽场景：早期版本写成 ``for sc in scenes`` 把整个场景池
    （512 个）都跑一遍，等于每轮 3 万个样本、显存里塞 512 的 batch，
    训练一轮要 32 秒还在偷偷改变有效学习率。
    """
    xs, ts, vs = [], [], []
    picks = rng.integers(0, len(scenes), size=int(batch))
    for k in picks:
        sc = scenes[int(k)]
        cloud = sd.sample_cloud(sc, rng,
                                density=corrupt["density"](),
                                outlier_ratio=corrupt["outlier"](),
                                noise_sigma=corrupt["noise"]())
        hm = sd.scene_to_input(sc, cloud, cells=cells)
        x = hm_mod.heightmap_channels(hm)
        t, v = sd.target_heatmap(sc, hm)
        xs.append(x)
        ts.append(t)
        vs.append(v.astype(np.float32))
    return (np.stack(xs), np.stack(ts)[:, None, :, :], np.stack(vs)[:, None, :, :])


def eval_argmax_error(model, scenes, rng, cells, device, n_batches=4, batch=24):
    """验证集落点误差（m）：用 numpy 前向 + 亚格精化，和部署端完全一致。"""
    model.eval()
    _NP_CACHE.clear()               # 权重每轮都变，缓存必须失效
    errs, hits = [], 0
    for _ in range(n_batches):
        for sc in scenes[:batch]:
            cloud = sd.sample_cloud(sc, rng, density=0.35, outlier_ratio=0.01, noise_sigma=0.03)
            hm = sd.scene_to_input(sc, cloud, cells=cells)
            heat = lcnn.forward(_np_model(model), hm_mod.heightmap_channels(hm))
            p = lcnn.heat_to_point(heat, hm, min_height_m=1.0, rough_max=ROUGH_MAX)
            if not p["ok"]:
                errs.append(9.99)
                continue
            d = float(np.hypot(p["x"] - sc.gt_xy[0], p["y"] - sc.gt_xy[1]))
            errs.append(d)
            hits += 1 if d <= 0.5 else 0
    errs = np.asarray(errs, dtype=np.float64)
    return {"mean_err_m": float(errs.mean()), "median_err_m": float(np.median(errs)),
            "p90_err_m": float(np.percentile(errs, 90)), "success_rate_0.5m": float(hits / len(errs)),
            "n": int(errs.size)}


_NP_CACHE = {}


def _np_model(model):
    """把 torch 权重转成 numpy 权重 dict（带缓存，避免每帧重复转换）。"""
    key = id(model)
    if key in _NP_CACHE:
        return _NP_CACHE[key]
    w = {}
    for name in LAYER_NAMES:
        layer = getattr(model, name)
        w[name + ".w"] = layer.weight.detach().cpu().numpy().astype(np.float32)
        w[name + ".b"] = layer.bias.detach().cpu().numpy().astype(np.float32)
    _NP_CACHE[key] = {"w": w, "meta": {}}
    return _NP_CACHE[key]


def parity_check(model, rng, cells, device, n=8):
    """numpy 前向 vs torch 前向：logits 最大偏差 + argmax 一致率。

    比的是 logits 而不是 sigmoid 之后的值：交叉熵训练会把 logits 拉得比较大，
    过 sigmoid 会把 1e-6 量级的浮点差放大成 1e-3，看起来吓人但没有意义；
    真正决定落点的是 argmax，所以两个指标一起报。
    """
    model.eval()
    _NP_CACHE.clear()               # 必须用最新权重做对齐
    diffs, agree = [], []
    with torch.no_grad():
        for _ in range(n):
            sc = sd.make_scene(rng)
            cloud = sd.sample_cloud(sc, rng, density=0.4, outlier_ratio=0.01, noise_sigma=0.03)
            hm = sd.scene_to_input(sc, cloud, cells=cells)
            x = hm_mod.heightmap_channels(hm)
            t_logit = model(torch.from_numpy(x)[None].to(device))[0, 0].cpu().numpy()
            n_logit = lcnn.forward(_np_model(model), x, return_heat=False)[0]
            diffs.append(float(np.max(np.abs(t_logit - n_logit))))
            agree.append(1.0 if int(np.argmax(t_logit)) == int(np.argmax(n_logit)) else 0.0)
    return {"max_logit_diff": float(np.max(diffs)), "argmax_agree": float(np.mean(agree))}


def write_loss_figure(rows, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                      # pragma: no cover
        print("[warn] matplotlib 不可用，跳过绘图: %s" % exc)
        return False
    ep = [r["epoch"] for r in rows]
    fig, ax = plt.subplots(1, 2, figsize=(10.0, 3.6))
    ax[0].plot(ep, [r["train_loss"] for r in rows], label="train")
    ax[0].plot(ep, [r["val_loss"] for r in rows], label="val")
    ax[0].set_xlabel("epoch")
    ax[0].set_ylabel("masked MSE")
    ax[0].set_title("loss")
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    ax[1].plot(ep, [r["val_err_m"] for r in rows], color="tab:red")
    ax[1].set_xlabel("epoch")
    ax[1].set_ylabel("landing error (m)")
    ax[1].set_title("val landing-point error")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    if not os.path.isdir(os.path.dirname(path)):
        os.makedirs(os.path.dirname(path))
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser(description="train the rooftop landing CNN")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batches-per-epoch", type=int, default=60)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--cells", type=int, default=64)
    ap.add_argument("--train-scenes", type=int, default=512)
    ap.add_argument("--val-scenes", type=int, default=128)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--seed", type=int, default=20261016)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--out", default=os.path.join(MODEL_DIR, "landing_cnn.npz"))
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    torch.manual_seed(args.seed)
    device = torch.device(args.device if (args.device != "cuda" or torch.cuda.is_available()) else "cpu")
    print("[env] python=%s torch=%s device=%s cuda=%s"
          % (sys.version.split()[0], torch.__version__, device,
             torch.version.cuda if torch.cuda.is_available() else "-"))

    train_scenes = [sd.make_scene(rng) for _ in range(args.train_scenes)]
    val_scenes = [sd.make_scene(rng) for _ in range(args.val_scenes)]

    # 训练时的点云退化范围（比评估时更宽，逼网络学会"稀疏 + 噪声 + 杂波"下的稳健性）
    corrupt = {
        "density": lambda: float(rng.uniform(0.05, 1.0)),
        "outlier": lambda: float(rng.uniform(0.0, 0.03)),
        "noise": lambda: float(rng.uniform(0.005, 0.05)),
    }

    model = LandingCNN().to(device)
    n_par = sum(p.numel() for p in model.parameters())
    print("[model] params=%d (=%.1f KB fp32)" % (n_par, n_par * 4 / 1024.0))
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)

    rows = []
    t_start = time.time()
    for epoch in range(1, args.epochs + 1):
        model.train()
        tot, nb = 0.0, 0
        for _ in range(args.batches_per_epoch):
            xs, ts, vs = build_batch(train_scenes, rng, args.cells, corrupt, batch=args.batch)
            x = torch.from_numpy(xs).to(device)
            t = torch.from_numpy(ts).to(device)
            v = torch.from_numpy(vs).to(device)
            prob = model(x)
            loss = soft_ce_loss(prob, t, v)
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += float(loss.item())
            nb += 1
        train_loss = tot / max(nb, 1)

        model.eval()
        with torch.no_grad():
            xs, ts, vs = build_batch(val_scenes, rng, args.cells, corrupt, batch=64)
            x = torch.from_numpy(xs).to(device)
            t = torch.from_numpy(ts).to(device)
            v = torch.from_numpy(vs).to(device)
            val_loss = float(soft_ce_loss(model(x), t, v).item())
        metrics = eval_argmax_error(model, val_scenes, rng, args.cells, device)
        rows.append({"epoch": epoch, "train_loss": train_loss, "val_loss": val_loss,
                     "val_err_m": metrics["mean_err_m"]})
        print("[epoch %2d/%d] train=%.5f val=%.5f val_err=%.3f m (%.1fs)"
              % (epoch, args.epochs, train_loss, val_loss, metrics["mean_err_m"], time.time() - t_start))
        sys.stdout.flush()

    parity = parity_check(model, rng, args.cells, device)
    print("[parity] max |torch - numpy| logits = %.3e   argmax agree = %.1f%%"
          % (parity["max_logit_diff"], parity["argmax_agree"] * 100.0))

    meta = {
        "version": lcnn.MODEL_VERSION,
        "cells": int(args.cells),
        "size_m": float(sd.ROI_SIZE_M),
        "min_height_m": 1.0,
        "rough_max": ROUGH_MAX,
        "channels": ["height", "mask", "rough"],
        "train_scenes": int(args.train_scenes),
        "epochs": int(args.epochs),
        "batch": int(args.batch),
        "batches_per_epoch": int(args.batches_per_epoch),
        "seed": int(args.seed),
        "torch": torch.__version__,
        "device": str(device),
        "val": metrics,
        "val_loss": rows[-1]["val_loss"],
        "numpy_torch_max_logit_diff": parity["max_logit_diff"],
        "numpy_torch_argmax_agree": parity["argmax_agree"],
        "train_seconds": round(time.time() - t_start, 1),
    }
    size = export_model(model, args.out, meta)

    if not os.path.isdir(RESULT_DIR):
        os.makedirs(RESULT_DIR)
    with open(os.path.join(RESULT_DIR, "loss.csv"), "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=["epoch", "train_loss", "val_loss", "val_err_m"])
        wr.writeheader()
        for r in rows:
            wr.writerow(r)
    with open(os.path.join(RESULT_DIR, "metrics.json"), "w") as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=2)

    drew = write_loss_figure(rows, os.path.join(FIG_DIR, "training_loss.png"))
    print("[out] model=%s (%.1f KB)  figure=%s" % (args.out, size / 1024.0, drew))
    print("[out] val: mean=%.3f m  median=%.3f m  p90=%.3f m  success@0.5m=%.1f%%"
          % (metrics["mean_err_m"], metrics["median_err_m"], metrics["p90_err_m"],
             metrics["success_rate_0.5m"] * 100.0))


if __name__ == "__main__":
    main()
