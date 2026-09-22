# -*- coding: utf-8 -*-
"""生成 CARLA 端到端 CNN 的训练/验证 MAE 曲线（供文档性能评价）。

用法（在装有 CARLA 环境、且已有采集数据集的机器上，可离线运行）：
    python scripts/gen_train_loss.py --data_dir dataset --epochs 30

产出 docs/assets/train_loss.png。
若尚无数据集，可先用端到端作业采集：
    python main.py --task end_to_end --mode collect --frames 300 --out_dir dataset
"""
import argparse
import os
import sys

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "04_end_to_end"))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data_dir", default="dataset")
    p.add_argument("--epochs", type=int, default=30)
    p.add_argument("--out", default=os.path.join(ROOT, "docs", "assets", "train_loss.png"))
    args = p.parse_args()

    import cv2  # noqa: E402
    img_dir = os.path.join(args.data_dir, "images")
    lbl_file = os.path.join(args.data_dir, "labels.txt")
    if not os.path.isdir(img_dir) or not os.path.isfile(lbl_file):
        print(f"[错误] 缺少 dataset：{args.data_dir}（需含 images/ 与 labels.txt）")
        return 1

    names = sorted(os.listdir(img_dir))
    with open(lbl_file) as f:
        labels = [float(l.split()[0]) for l in f]
    if len(labels) != len(names):
        print(f"[错误] 标签({len(labels)})与图像({len(names)})数不一致"); return 1

    X, Y = [], []
    for i, name in enumerate(names):
        img = cv2.imread(os.path.join(img_dir, name))
        if img is None:
            continue
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
        X.append(img); Y.append(labels[i])
    X = np.asarray(X); Y = np.asarray(Y).reshape(-1, 1)
    print(f"数据集 X{X.shape} Y{Y.shape}")

    from main import build_cnn  # 端到端模块里的 CNN
    cnn = build_cnn()
    import tensorflow as tf
    history = cnn.fit(X, Y, epochs=args.epochs, batch_size=32,
                      validation_split=0.15, verbose=1)
    tm = history.history["mae"]; vm = history.history["val_mae"]
    print(f"train_mae最后={tm[-1]:.4f}  val_mae最后={vm[-1]:.4f}")

    # 绘图（纯 PIL，无 matplotlib 依赖）
    from PIL import Image, ImageDraw
    W, H = 800, 480
    img_plot = Image.new("RGB", (W, H), "white")
    dr = ImageDraw.Draw(img_plot)
    pl, pr, pt, pb = 70, 40, 50, 60
    w, h = W - pl - pr, H - pt - pb
    n = len(tm)
    ymax = max(max(tm), max(vm)) * 1.1

    def xy(i, v):
        return (pl + w * i / max(n - 1, 1), pt + h * (1 - v / ymax))

    for k in range(n - 1):
        dr.line([xy(k, tm[k]), xy(k + 1, tm[k + 1])], fill="blue", width=3)
        dr.line([xy(k, vm[k]), xy(k + 1, vm[k + 1])], fill="red", width=3)
    for i in range(n):
        x, y = xy(i, tm[i]); dr.ellipse([x - 3, y - 3, x + 3, y + 3], fill="blue")
        x2, y2 = xy(i, vm[i]); dr.ellipse([x2 - 3, y2 - 3, x2 + 3, y2 + 3], fill="red")
    for j in range(5):
        gv = ymax * j / 4
        y = pt + h * (1 - gv / ymax)
        dr.line([(pl, y), (W - pr, y)], fill="lightgray")
        dr.text((10, y - 8), f"{gv:.2f}", fill="black")
    dr.text((pl, 6), "CARLA end-to-end CNN MAE (blue=train, red=val)", fill="black")
    dr.text((pl, H - pb + 16), "Epoch", fill="black")
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    img_plot.save(args.out)
    print("已保存:", args.out, os.path.getsize(args.out), "bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
