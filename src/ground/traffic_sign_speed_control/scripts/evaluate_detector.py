# -*- coding: utf-8 -*-
"""字符级退化矩阵评测。

对 0-9 每个数字在 {clean, blur, dark, rotate, noise, 组合} 六种退化条件下
各生成 N 张合成样本，统计每种条件下的准确率，输出
artifacts/detector_eval.json 与退化矩阵热图 artifacts/degrade_matrix.png。
"""
import argparse
import json
import random
import sys
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from traffic_sign_speed_control.model import DigitCNN, load_model  # noqa: E402
from traffic_sign_speed_control.sign_render import (  # noqa: E402
    augment, canonicalize_char, _load_font)
from PIL import Image, ImageDraw  # noqa: E402

CONDITIONS = {
    "clean":     {},
    "blur":      {"blur_sigma": 1.4},
    "dark":      {"brightness": 0.45},
    "rotate":    {"rotate": 14},
    "noise":     {"noise_std": 0.07},
    "combined":  {"blur_sigma": 1.2, "brightness": 0.35, "rotate": 12, "noise_std": 0.05},
}


def render_digit(digit, rng):
    big = 96
    canvas = Image.new("RGB", (big, big), (255, 255, 255))
    d = ImageDraw.Draw(canvas)
    font = _load_font(int(big * rng.uniform(0.55, 0.75)))
    bbox = d.textbbox((0, 0), str(digit), font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    d.text(((big - tw) / 2 - bbox[0], (big - th) / 2 - bbox[1]),
           str(digit), font=font, fill=(rng.randint(0, 60),) * 3)
    if rng.random() < 0.5:
        r = big // 2 - rng.randint(0, 3)
        d.ellipse([big // 2 - r, big // 2 - r, big // 2 + r, big // 2 + r],
                  outline=(200, 16, 26), width=rng.randint(2, 6))
    return canvas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-per-cond", type=int, default=100)
    ap.add_argument("--seed", type=int, default=123)
    args = ap.parse_args()

    import torch
    model, device = load_model()
    rng = np.random.default_rng(args.seed)
    results = {}
    for cond, kw in CONDITIONS.items():
        correct = 0
        total = 0
        xs, ys = [], []
        for digit in range(10):
            for _ in range(args.n_per_cond):
                img = augment(render_digit(digit, random.Random(
                    int(rng.integers(0, 2**31)))), None, **kw)
                g = canonicalize_char(np.asarray(img.convert("L")), size=32)
                if g is None:
                    continue
                xs.append(g.astype(np.float32) / 255.0)
                ys.append(digit)
        x = torch.from_numpy(np.stack(xs)[:, None])
        with torch.no_grad():
            pred = model(x.to(device)).argmax(1).cpu().numpy()
        correct = int((pred == np.array(ys)).sum())
        total = len(ys)
        results[cond] = {"acc": correct / total, "n": total}
        print(f"  {cond:<9}: acc = {correct}/{total} = {correct / total:.4f}")

    art = PKG / "artifacts"
    art.mkdir(exist_ok=True)
    (art / "detector_eval.json").write_text(
        json.dumps(results, indent=2, ensure_ascii=False))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 3))
    names = list(results)
    ax.bar(names, [results[c]["acc"] for c in names], color="#cc4444")
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("accuracy")
    ax.set_title("char accuracy under degradations")
    for i, c in enumerate(names):
        ax.text(i, results[c]["acc"] + 0.02, f"{results[c]['acc']:.3f}",
                ha="center", fontsize=8)
    fig.tight_layout()
    fig.savefig(art / "degrade_matrix.png", dpi=120)
    print(f"[done] 已写入 {art / 'detector_eval.json'}")


if __name__ == "__main__":
    main()
