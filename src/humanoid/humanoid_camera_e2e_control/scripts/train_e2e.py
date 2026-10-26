# -*- coding: utf-8 -*-
"""
train_e2e.py —— 训练人形机器人图像端到端 CNN（standalone，无 ROS）。

读取 collect_data.py 采集的图像—指令数据集，划分训练/测试集，在归一化指令
空间用 MSE 损失训练 E2ECNN；训练结束在测试集上评估指令回归误差与方向分类
准确率，保存权重与评测指标 JSON。

运行（包根目录）：
    python3 scripts/train_e2e.py
    python3 scripts/train_e2e.py --epochs 50 --batch-size 32
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PKG_ROOT)

from humanoid_camera_e2e_control.sim_core import HumanoidSim
from humanoid_camera_e2e_control.network import E2ECNN
from humanoid_camera_e2e_control.data_io import load_dataset, DrivingDataset, direction_label


def main():
    ap = argparse.ArgumentParser(description="训练图像端到端策略 CNN")
    here = _PKG_ROOT
    ap.add_argument("--data-dir", default=os.path.join(here, "data", "training"))
    ap.add_argument("--weights", default=os.path.join(here, "weights", "humanoid_cnn.pt"))
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--batch-size", type=int, default=32)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[训练] 设备：{device} | 数据目录：{args.data_dir}")

    packed = load_dataset(args.data_dir)
    if packed is None:
        print("[错误] 未找到数据集，请先运行 scripts/collect_data.py 采集数据。")
        sys.exit(1)
    images, labels = packed
    n = len(images)
    print(f"[训练] 样本数：{n} | 图像 {images.shape[1:]} | 指令范围 "
          f"v=[{labels[:,0].min():.2f},{labels[:,0].max():.2f}] "
          f"w=[{labels[:,1].min():.2f},{labels[:,1].max():.2f}]")

    v_max, w_max = HumanoidSim.V_MAX, HumanoidSim.W_MAX
    ds = DrivingDataset(images, labels, v_max, w_max)
    n_train = int(0.85 * n)
    n_test = n - n_train
    train_ds, test_ds = random_split(ds, [n_train, n_test],
                                     generator=torch.Generator().manual_seed(args.seed))
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    test_loader = DataLoader(test_ds, batch_size=args.batch_size, shuffle=False)

    net = E2ECNN(v_max, w_max).to(device)
    # 初始化 LazyLinear 形状
    net(torch.zeros(1, 3, 96, 128).to(device))
    opt = torch.optim.Adam(net.parameters(), lr=args.lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    loss_fn = nn.MSELoss()
    cls_fn = nn.CrossEntropyLoss()
    # 转向（w）是避障关键，回归损失加权；分类辅助头以交叉熵帮助学到左右判别特征
    w_weight = 2.0
    cls_weight = 1.0

    history = {"train_loss": [], "test_loss": [], "train_w": [], "test_w": [],
               "train_cls_acc": [], "test_cls_acc": []}
    t0 = time.time()
    for epoch in range(1, args.epochs + 1):
        net.train()
        train_loss = train_w = train_cls_correct = 0.0
        for x, y, c in train_loader:
            x, y, c = x.to(device), y.to(device), c.to(device)
            opt.zero_grad()
            pred = net.forward_norm(x)
            logits = net.forward_class(x)
            loss_v = loss_fn(pred[:, 0], y[:, 0])
            loss_w = loss_fn(pred[:, 1], y[:, 1])
            loss_reg = loss_v + w_weight * loss_w
            loss_cls = cls_fn(logits, c)
            loss = loss_reg + cls_weight * loss_cls
            loss.backward()
            opt.step()
            train_loss += loss_reg.item() * len(x)
            train_w += loss_w.item() * len(x)
            train_cls_correct += (logits.argmax(1) == c).sum().item()
        train_loss /= n_train
        train_w /= n_train
        train_cls_acc = train_cls_correct / n_train

        net.eval()
        test_loss = test_w = test_cls_correct = 0.0
        with torch.no_grad():
            for x, y, c in test_loader:
                x, y, c = x.to(device), y.to(device), c.to(device)
                pred = net.forward_norm(x)
                logits = net.forward_class(x)
                loss_v = loss_fn(pred[:, 0], y[:, 0])
                loss_w = loss_fn(pred[:, 1], y[:, 1])
                test_loss += (loss_v + w_weight * loss_w).item() * len(x)
                test_w += loss_w.item() * len(x)
                test_cls_correct += (logits.argmax(1) == c).sum().item()
        test_loss /= n_test
        test_w /= n_test
        test_cls_acc = test_cls_correct / n_test
        history["train_loss"].append(round(train_loss, 5))
        history["test_loss"].append(round(test_loss, 5))
        history["train_w"].append(round(train_w, 5))
        history["test_w"].append(round(test_w, 5))
        history["train_cls_acc"].append(round(train_cls_acc, 4))
        history["test_cls_acc"].append(round(test_cls_acc, 4))
        if epoch % 5 == 0 or epoch == 1:
            print(f"[Epoch {epoch:2d}/{args.epochs}] train MSE={train_loss:.4f} "
                  f"test MSE={test_loss:.4f} | train_w={train_w:.4f} test_w={test_w:.4f} "
                  f"| cls train={train_cls_acc:.3f} test={test_cls_acc:.3f}")
        sched.step()

    train_time = time.time() - t0

    # ---------------- 测试集评测 ----------------
    net.eval()
    preds_v, preds_w, gts_v, gts_w = [], [], [], []
    correct, cls_correct, total = 0, 0, 0
    with torch.no_grad():
        for x, y, c in test_loader:
            x, y, c = x.to(device), y.to(device), c.to(device)
            pred = net(x)  # 物理指令（回归头）
            logits = net.forward_class(x)  # 方向分类头
            preds_v.append(pred[:, 0].cpu().numpy())
            preds_w.append(pred[:, 1].cpu().numpy())
            gts_v.append(y[:, 0].cpu().numpy() * v_max)
            gts_w.append(y[:, 1].cpu().numpy() * w_max)
            cls_correct += (logits.argmax(1) == c).sum().item()
            for i in range(len(x)):
                gt_dir = direction_label(y[i, 0].item() * v_max, y[i, 1].item() * w_max)
                pd_dir = direction_label(pred[i, 0].item(), pred[i, 1].item())
                correct += int(gt_dir == pd_dir)
                total += 1

    preds_v = np.concatenate(preds_v); preds_w = np.concatenate(preds_w)
    gts_v = np.concatenate(gts_v); gts_w = np.concatenate(gts_w)
    metrics = {
        "samples": n,
        "train_samples": n_train,
        "test_samples": n_test,
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "lr": args.lr,
        "device": device,
        "train_time_s": round(train_time, 1),
        "final_train_mse": history["train_loss"][-1],
        "final_test_mse": history["test_loss"][-1],
        "final_train_w": history["train_w"][-1],
        "final_test_w": history["test_w"][-1],
        "rmse_v_mps": round(float(np.sqrt(np.mean((preds_v - gts_v) ** 2))), 4),
        "rmse_w_rads": round(float(np.sqrt(np.mean((preds_w - gts_w) ** 2))), 4),
        "direction_accuracy": round(correct / total, 4),
        "class_accuracy": round(cls_correct / total, 4),
        "direction_total": total,
        "history": history,
    }

    net.save(args.weights)
    metrics_path = os.path.join(os.path.dirname(args.weights), "metrics.json")
    with open(metrics_path, "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)

    print("\n================ 测试集评测 ================")
    print(f"测试集样本        : {n_test}")
    print(f"最终 train MSE    : {metrics['final_train_mse']}")
    print(f"最终 test  MSE    : {metrics['final_test_mse']}")
    print(f"线速度 RMSE (m/s) : {metrics['rmse_v_mps']}")
    print(f"偏航角 RMSE (rad/s): {metrics['rmse_w_rads']}")
    print(f"方向三态准确率    : {metrics['direction_accuracy']*100:.1f}% "
          f"({correct}/{total})")
    print(f"分类头三态准确率  : {metrics['class_accuracy']*100:.1f}% "
          f"({cls_correct}/{total})")
    print(f"[保存] 权重 -> {args.weights}")
    print(f"[保存] 指标 -> {metrics_path}")


if __name__ == "__main__":
    main()
