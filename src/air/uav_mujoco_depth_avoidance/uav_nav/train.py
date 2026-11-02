"""训练、评估与出图。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .data import make_dataset
from .env import DepthUavEnv
from .net import DepthCNN


def train(env: DepthUavEnv, epochs: int = 14, n_train: int = 900, n_val: int = 200,
          batch: int = 32, lr: float = 2e-3, seed: int = 0, log=print):
    x_tr, y_tr = make_dataset(env, n_train, seed=seed)
    x_va, y_va = make_dataset(env, n_val, seed=seed + 1000)
    net = DepthCNN(seed=seed)
    rng = np.random.default_rng(seed)
    history = []
    for epoch in range(epochs):
        order = rng.permutation(n_train)
        losses = []
        for start in range(0, n_train, batch):
            idx = order[start:start + batch]
            pred, cache = net.forward(x_tr[idx])
            grads = net.backward(cache, y_tr[idx])
            net.step(grads, lr)
            losses.append(float(np.mean((pred - y_tr[idx]) ** 2)))
        val = net.predict(x_va)
        mae = np.abs(val - y_va).mean(axis=0)
        row = {
            "epoch": epoch + 1,
            "train_mse": round(float(np.mean(losses)), 6),
            "val_mae_left": round(float(mae[0]), 4),
            "val_mae_center": round(float(mae[1]), 4),
            "val_mae_right": round(float(mae[2]), 4),
        }
        history.append(row)
        log("epoch {epoch:>2}/{epochs}  mse={train_mse:.5f}  "
            "MAE(l/c/r)={val_mae_left:.3f}/{val_mae_center:.3f}/{val_mae_right:.3f}".format(
                epochs=epochs, **row))
    return net, history, (x_va, y_va)


def plot_training(history, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ep = [h["epoch"] for h in history]
    fig, ax1 = plt.subplots(figsize=(8, 4))
    ax1.plot(ep, [h["train_mse"] for h in history], "o-", color="#2f80ed", label="train MSE")
    ax1.set_xlabel("epoch")
    ax1.set_ylabel("train MSE", color="#2f80ed")
    ax1.tick_params(axis="y", labelcolor="#2f80ed")
    ax2 = ax1.twinx()
    for key, color, marker in (("val_mae_left", "#eb5757", "s--"),
                               ("val_mae_center", "#27ae60", "^:"),
                               ("val_mae_right", "#9b51e0", "d-.")):
        ax2.plot(ep, [h[key] for h in history], marker, color=color, label=key.replace("val_mae_", ""))
    ax2.set_ylabel("validation MAE (clearance)")
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [l.get_label() for l in lines], loc="upper right", fontsize=8)
    ax1.grid(alpha=0.3)
    fig.suptitle("depth clearance CNN training (numpy only)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_prediction(net: DepthCNN, x_va, y_va, path: Path, count: int = 4) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pred = net.predict(x_va[:count])
    fig, axes = plt.subplots(2, count, figsize=(2.2 * count, 4.6))
    for i in range(count):
        axes[0, i].imshow(1.0 - x_va[i, :, :, 0], cmap="viridis")
        axes[0, i].set_title("depth (32x32)", fontsize=8)
        axes[0, i].axis("off")
        names = ["left", "center", "right"]
        axes[1, i].bar(names, y_va[i], color="#2f80ed", alpha=0.7, label="truth")
        axes[1, i].plot(names, pred[i], "ro", label="pred")
        axes[1, i].set_ylim(0, 1.05)
        axes[1, i].tick_params(labelsize=7)
        if i == 0:
            axes[1, i].legend(fontsize=7)
    fig.suptitle("depth image -> sector clearance (truth vs prediction)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def save_gif(frames, path: Path, duration: int = 90) -> tuple[int, int]:
    from PIL import Image, ImageDraw

    images = []
    for rgb, depth, text in frames:
        base = Image.fromarray(rgb).resize((192, 192), Image.BILINEAR)
        dep = Image.fromarray((np.clip(depth / 8.0, 0, 1) * 255).astype(np.uint8))
        dep = dep.resize((192, 192), Image.BILINEAR).convert("RGB")
        canvas = Image.new("RGB", (392, 214), (18, 20, 26))
        canvas.paste(base, (0, 22))
        canvas.paste(dep, (200, 22))
        draw = ImageDraw.Draw(canvas)
        draw.text((4, 5), "chase view", fill=(200, 210, 225))
        draw.text((204, 5), "depth cam", fill=(200, 210, 225))
        draw.text((4, 216 - 18), text, fill=(235, 235, 235))
        images.append(canvas)
    images[0].save(path, save_all=True, append_images=images[1:], duration=duration, loop=0)
    return path.stat().st_size, len(images)


def save_report(history, runs, out_dir) -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    summary = {"training": history, "closed_loop": runs}
    (out / "metrics.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False),
                                      encoding="utf-8")
    return summary
