"""Training, evaluation, figures and a closed-loop driving demo."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .cnn import TinyCNN
from .dataset import IMG, make_dataset, render, road_profile

STEER_GAIN = 2.0      # m/s of lateral motion per unit steering
DT = 0.1
CURV_DRIFT = 8.0


def train(epochs: int = 12, n_train: int = 1200, n_val: int = 300, batch: int = 32,
          lr: float = 1e-3, seed: int = 0, log=print):
    xtr, ytr, _ = make_dataset(n_train, seed=seed)
    xva, yva, _ = make_dataset(n_val, seed=seed + 1)
    net = TinyCNN(seed=seed)
    rng = np.random.default_rng(seed)
    history = []

    for epoch in range(epochs):
        order = rng.permutation(n_train)
        losses = []
        for start in range(0, n_train, batch):
            idx = order[start:start + batch]
            pred, cache = net.forward(xtr[idx])
            grads = net.backward(cache, ytr[idx])
            net.step(grads, lr)
            losses.append(float(np.mean((pred - ytr[idx]) ** 2)))
        val = net.predict(xva)
        mae = np.abs(val - yva).mean(axis=0)
        row = {
            "epoch": epoch + 1,
            "train_mse": round(float(np.mean(losses)), 6),
            "val_mae_steering": round(float(mae[0]), 5),
            "val_mae_throttle": round(float(mae[1]), 5),
        }
        history.append(row)
        log("epoch {epoch:>2}/{epochs}  mse={train_mse:.5f}  "
            "MAE(steer)={val_mae_steering:.4f}  MAE(throttle)={val_mae_throttle:.4f}".format(
                epochs=epochs, **row))
    return net, history, (xva, yva)


def closed_loop_drive(net, steps: int = 120, lateral0: float = 0.9):
    """Drive the S-curve with the network in the loop; collect frames + metrics."""
    rng = np.random.default_rng(7)
    lateral = lateral0
    errors, steers, frames = [], [], []
    for i in range(steps):
        curvature, offset = road_profile(i, steps)
        rel_offset = offset - lateral           # road centre as seen by the camera
        image = render(curvature, rel_offset, rng, noise=0.0)
        pred = net.predict(image[None, :, :, None])[0]
        steer, throttle = float(pred[0]), float(pred[1])

        lateral += (CURV_DRIFT * curvature - STEER_GAIN * steer) * DT
        lateral = float(np.clip(lateral, -3.0, 3.0))

        errors.append(abs(offset - lateral))
        steers.append(steer)
        frames.append((image.copy(), steer, throttle, offset - lateral))

    errors = np.asarray(errors, dtype=np.float32)
    metrics = {
        "steps": steps,
        "mean_abs_lateral_error_m": round(float(errors.mean()), 4),
        "max_abs_lateral_error_m": round(float(errors.max()), 4),
        "final_abs_lateral_error_m": round(float(errors[-1]), 4),
        "within_lane_ratio": round(float((errors < 0.6).mean()), 4),
        "mean_abs_steering": round(float(np.mean(np.abs(steers))), 4),
    }
    return frames, metrics


def plot_training(history, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    epochs = [h["epoch"] for h in history]
    fig, ax1 = plt.subplots(figsize=(8, 4.2))
    ax1.plot(epochs, [h["train_mse"] for h in history], "o-", color="#2f80ed", label="train MSE")
    ax1.set_xlabel("epoch")
    ax1.set_ylabel("train MSE", color="#2f80ed")
    ax1.tick_params(axis="y", labelcolor="#2f80ed")
    ax2 = ax1.twinx()
    ax2.plot(epochs, [h["val_mae_steering"] for h in history], "s--", color="#eb5757",
             label="val MAE steering")
    ax2.plot(epochs, [h["val_mae_throttle"] for h in history], "^:", color="#27ae60",
             label="val MAE throttle")
    ax2.set_ylabel("validation MAE")
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [l.get_label() for l in lines], loc="upper right", fontsize=8)
    ax1.grid(alpha=0.3)
    fig.suptitle("pix2cmd CNN training (numpy only)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_predictions(net, xva, yva, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pred = net.predict(xva)
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.8))
    axes[0].scatter(yva[:, 0], pred[:, 0], s=6, alpha=0.5, color="#2f80ed")
    lim = [-1.05, 1.05]
    axes[0].plot(lim, lim, "k--", linewidth=1)
    axes[0].set_xlabel("true steering")
    axes[0].set_ylabel("predicted steering")
    axes[0].set_title("steering (MAE=%.3f)" % float(np.abs(pred[:, 0] - yva[:, 0]).mean()))
    axes[0].grid(alpha=0.3)

    axes[1].scatter(yva[:, 1], pred[:, 1], s=6, alpha=0.5, color="#27ae60")
    axes[1].plot([0, 1], [0, 1], "k--", linewidth=1)
    axes[1].set_xlabel("true throttle")
    axes[1].set_ylabel("predicted throttle")
    axes[1].set_title("throttle (MAE=%.3f)" % float(np.abs(pred[:, 1] - yva[:, 1]).mean()))
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_samples(net, xva, yva, path: Path, count: int = 6) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pred = net.predict(xva[:count])
    fig, axes = plt.subplots(1, count, figsize=(2.0 * count, 2.4))
    for i, ax in enumerate(axes):
        ax.imshow(xva[i, :, :, 0], cmap="gray", vmin=0, vmax=1)
        ax.set_title("true %.2f\npred %.2f" % (yva[i, 0], pred[i, 0]), fontsize=8)
        ax.axis("off")
    fig.suptitle("camera input -> predicted steering", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def save_gif(frames, path: Path, scale: int = 3, duration: int = 70) -> tuple[int, int]:
    """Compose the recorded frames into an animated GIF with a steering HUD."""
    from PIL import Image, ImageDraw

    size = IMG * scale
    width = size + 90
    images = []
    for image, steer, throttle, error in frames:
        rgb = np.repeat((image * 255).astype(np.uint8)[:, :, None], 3, axis=2)
        base = Image.fromarray(rgb).resize((size, size), Image.NEAREST)
        canvas = Image.new("RGB", (width, size), (18, 20, 26))
        canvas.paste(base, (0, 0))
        draw = ImageDraw.Draw(canvas)
        x0 = size + 10
        draw.text((x0, 6), "steering", fill=(200, 210, 225))
        draw.rectangle([x0, 20, x0 + 70, 34], outline=(90, 100, 120))
        mid = x0 + 35
        draw.rectangle([min(mid, mid + int(steer * 35)), 21,
                        max(mid, mid + int(steer * 35)), 33], fill=(72, 160, 255))
        draw.text((x0, 46), "throttle", fill=(200, 210, 225))
        draw.rectangle([x0, 60, x0 + 70, 74], outline=(90, 100, 120))
        draw.rectangle([x0, 61, x0 + int(throttle * 70), 73], fill=(39, 174, 96))
        draw.text((x0, 86), "error", fill=(200, 210, 225))
        draw.text((x0, 100), "%.2f m" % error, fill=(235, 87, 87))
        images.append(canvas)

    images[0].save(path, save_all=True, append_images=images[1:], duration=duration, loop=0)
    return path.stat().st_size, len(images)


def save_report(history, metrics, out_dir: str | Path) -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    summary = {"training": history, "closed_loop": metrics}
    (out / "metrics.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False),
                                      encoding="utf-8")
    return summary
