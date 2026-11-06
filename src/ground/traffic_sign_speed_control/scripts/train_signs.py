# -*- coding: utf-8 -*-
"""合成数据生成 + DigitCNN 训练。

用法：python3 scripts/train_signs.py [--epochs 15] [--n-per-class 400]
输出：models/sign_cnn.pth、artifacts/train_metrics.json、
      artifacts/train_curve.png、artifacts/confusion_matrix.png、
      artifacts/sample_grid.png（文档配图）
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from traffic_sign_speed_control.model import DigitCNN  # noqa: E402
from traffic_sign_speed_control.sign_render import render_char_dataset  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=15)
    ap.add_argument("--n-per-class", type=int, default=400)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    import torch
    import torch.nn as nn

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = "cuda" if torch.cuda.is_available() else "cpu"

    print(f"[data] 生成合成字符数据集 (每类 {args.n_per_class} 张) ...")
    x, y = render_char_dataset(n_per_class=args.n_per_class, size=32, seed=args.seed)
    n = len(y)
    idx = np.random.permutation(n)
    n_tr, n_va = int(0.8 * n), int(0.1 * n)
    tr, va, te = idx[:n_tr], idx[n_tr:n_tr + n_va], idx[n_tr + n_va:]
    x_t = torch.from_numpy(x)
    y_t = torch.from_numpy(y)
    print(f"[data] total={n} train={len(tr)} val={len(va)} test={len(te)}")

    model = DigitCNN().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.CrossEntropyLoss()
    bs = 128
    hist = {"train_loss": [], "val_loss": [], "train_acc": [], "val_acc": []}

    def eval_split(ixs):
        model.eval()
        correct = total = 0
        with torch.no_grad():
            for k in range(0, len(ixs), 512):
                xb = x_t[ixs[k:k + 512]].to(device)
                yb = y_t[ixs[k:k + 512]].to(device)
                pred = model(xb).argmax(1)
                correct += int((pred == yb).sum())
                total += len(yb)
        return correct / max(total, 1)

    print("[train] 开始训练 ...")
    for ep in range(args.epochs):
        model.train()
        perm = torch.randperm(len(tr))
        run_loss = run_correct = 0
        for k in range(0, len(tr), bs):
            sel = torch.from_numpy(tr)[perm[k:k + bs]]
            xb, yb = x_t[sel].to(device), y_t[sel].to(device)
            opt.zero_grad()
            out = model(xb)
            loss = loss_fn(out, yb)
            loss.backward()
            opt.step()
            run_loss += float(loss) * len(sel)
            run_correct += int((out.argmax(1) == yb).sum())
        tr_loss = run_loss / len(tr)
        tr_acc = run_correct / len(tr)
        va_acc = eval_split(va)
        # 验证损失
        model.eval()
        with torch.no_grad():
            vb = x_t[va].to(device)
            yv = y_t[va].to(device)
            va_loss = float(loss_fn(model(vb), yv))
        hist["train_loss"].append(tr_loss)
        hist["val_loss"].append(va_loss)
        hist["train_acc"].append(tr_acc)
        hist["val_acc"].append(va_acc)
        print(f"  epoch {ep + 1:02d}: train_loss={tr_loss:.4f} acc={tr_acc:.4f} "
              f"val_loss={va_loss:.4f} acc={va_acc:.4f}")

    te_acc = eval_split(te)
    print(f"[test] 测试集准确率 = {te_acc:.4f}")

    # 混淆矩阵
    model.eval()
    with torch.no_grad():
        pred = model(x_t[te].to(device)).argmax(1).cpu().numpy()
    cm = np.zeros((10, 10), int)
    for t, p in zip(y[te], pred):
        cm[int(t), int(p)] += 1
    print("[cm] 混淆矩阵对角线:", np.diag(cm).tolist())

    # 保存权重与指标
    (PKG / "models").mkdir(exist_ok=True)
    torch.save(model.state_dict(), PKG / "models" / "sign_cnn.pth")
    art = PKG / "artifacts"
    art.mkdir(exist_ok=True)
    (art / "train_metrics.json").write_text(json.dumps({
        "epochs": args.epochs, "n_per_class": args.n_per_class,
        "n_total": int(n), "test_acc": te_acc,
        "history": hist, "confusion_matrix": cm.tolist(),
    }, indent=2, ensure_ascii=False))

    # 绘图（训练曲线 / 混淆矩阵 / 样本网格）
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 2, figsize=(9, 3.2))
    ax[0].plot(hist["train_loss"], label="train")
    ax[0].plot(hist["val_loss"], label="val")
    ax[0].set_title("Loss"); ax[0].set_xlabel("epoch"); ax[0].legend()
    ax[1].plot(hist["train_acc"], label="train")
    ax[1].plot(hist["val_acc"], label="val")
    ax[1].set_title("Accuracy"); ax[1].set_xlabel("epoch"); ax[1].legend()
    fig.tight_layout()
    fig.savefig(art / "train_curve.png", dpi=120)

    fig, ax = plt.subplots(figsize=(5, 4))
    im = ax.imshow(cm, cmap="Blues")
    ax.set_xticks(range(10)); ax.set_yticks(range(10))
    ax.set_xlabel("pred"); ax.set_ylabel("true")
    fig.colorbar(im)
    fig.tight_layout()
    fig.savefig(art / "confusion_matrix.png", dpi=120)

    from traffic_sign_speed_control.sign_render import render_char_dataset as rc
    xs, ys = rc(n_per_class=8, size=32, seed=7)
    fig, axes = plt.subplots(2, 10, figsize=(10, 2.2))
    for i, ax_i in enumerate(axes.flat):
        ax_i.imshow(xs[i, 0], cmap="gray", vmin=0, vmax=1)
        ax_i.set_title(str(ys[i]), fontsize=8)
        ax_i.axis("off")
    fig.suptitle("synthetic char samples", fontsize=10)
    fig.tight_layout()
    fig.savefig(art / "sample_grid.png", dpi=120)
    print(f"[done] 权重与指标已写入 {PKG / 'models'} 和 {art}")


if __name__ == "__main__":
    main()
