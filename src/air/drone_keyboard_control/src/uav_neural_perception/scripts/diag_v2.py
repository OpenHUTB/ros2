#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线诊断：加载训练权重，遍历数据集，评估各动作方向/幅值与回归质量。

模型与数据集均在归一化 [-1,1] 空间训练；本脚本在归一化空间检查合法性，
再反归一化回原始指令尺度（vx±2.0 / vy±0.5 / yaw±0.4）评估方向与幅值。
"""
import os, sys, collections
import numpy as np
import torch
from torch.utils.data import DataLoader

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from dataset import ImageDataset
from model import build_model

WEIGHT = os.path.join(HERE, "models", "best_drone_model.pth")
CSV = os.path.expanduser("~/perception_dataset/data.csv")
MAX_VX, MAX_VY, MAX_YAW = 2.0, 0.5, 0.4
SCALE = np.array([MAX_VX, MAX_VY, MAX_YAW])


def main():
    device = torch.device("cpu")
    ds = ImageDataset(CSV, augment=False)
    print(f"样本总数: {len(ds)}")
    loader = DataLoader(ds, batch_size=16, shuffle=False)

    model = build_model().to(device)
    ckpt = torch.load(WEIGHT, map_location=device)
    state = ckpt.get("state_dict", ckpt)
    model.load_state_dict(state, strict=True)
    model.eval()

    preds_n, labels_n = [], []
    with torch.no_grad():
        for imgs, acts in loader:
            out = model(imgs.to(device)).cpu().numpy()
            preds_n.append(out)
            labels_n.append(acts.numpy())
    preds_n = np.concatenate(preds_n)
    labels_n = np.concatenate(labels_n)

    # 合法性：归一化空间（tanh 约束在 [-1,1]，不得出现 NaN/inf）
    finite = bool(np.isfinite(preds_n).all())
    within_norm = bool(np.abs(preds_n).max() <= 1.0 + 1e-6)
    print(f"\n预测全部有限(无NaN/inf): {finite}; 归一化输出均在[-1,1]内: {within_norm}")

    # 反归一化到原始指令尺度，后续统计均基于原始尺度
    preds = preds_n * SCALE
    labels = labels_n * SCALE

    names = ["vx", "vy", "yaw"]
    print("\n===== 整体回归质量（原始尺度）=====")
    print(f"{'通道':<6}{'标签均值':>10}{'标签std':>10}{'预测均值':>10}{'预测std':>10}{'MSE':>10}")
    for i, n in enumerate(names):
        mse = np.mean((preds[:, i] - labels[:, i]) ** 2)
        print(f"{n:<6}{labels[:,i].mean():10.3f}{labels[:,i].std():10.3f}"
              f"{preds[:,i].mean():10.3f}{preds[:,i].std():10.3f}{mse:10.4f}")

    # 分动作类别（key 取原始标签）
    groups = collections.defaultdict(list)
    for p, l in zip(preds, labels):
        key = tuple(np.round(l, 1))
        groups[key].append((p, l))

    print("\n===== 分动作诊断 =====")
    print(f"{'动作(标签 vx,vy,yaw)':<28}{'n':>5}{'预测均值':>26}{'方向一致':>8}")
    for key, items in sorted(groups.items(), key=lambda kv: -len(kv[1])):
        arr_p = np.array([x[0] for x in items])
        arr_l = np.array([x[1] for x in items])
        pred_mean = arr_p.mean(axis=0)
        n = len(items)
        agree = []
        for i in range(3):
            if abs(arr_l[:, i].mean()) > 1e-3:
                same = np.sign(arr_p[:, i]) == np.sign(arr_l[:, i])
                agree.append(same.mean())
        agree_rate = np.mean(agree) if agree else 1.0
        pm = "(" + ", ".join(f"{v:+.2f}" for v in pred_mean) + ")"
        print(f"{str(key):<28}{n:>5}{pm:>26}{agree_rate*100:7.1f}%")

    # 重点：转向方向召回
    print("\n===== 转向专项 =====")
    for yaw_label, tag in [(-0.4, "左转"), (0.4, "右转")]:
        idx = np.where(np.round(labels[:, 2], 1) == yaw_label)[0]
        if len(idx) == 0:
            print(f"{tag}: 无样本")
            continue
        pred_yaw = preds[idx, 2]
        correct = np.mean(np.sign(pred_yaw) == np.sign(labels[idx, 2]))
        print(f"{tag} n={len(idx):<4} 预测yaw均值={pred_yaw.mean():+.3f} "
              f"方向一致率={correct*100:.1f}%")


if __name__ == "__main__":
    main()
