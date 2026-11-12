"""训练（模仿 MPPI）+ 出图。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .env import AckermannEnv
from .nav import collect_expert  # noqa: F401  (对外导出)
from .net import TrackerNet


def train(net: TrackerNet, x: np.ndarray, y: np.ndarray, epochs: int = 120,
          batch: int = 64, lr: float = 2e-3, seed: int = 0, log=print):
    rng = np.random.default_rng(seed)
    hist = []
    for ep in range(epochs):
        order = rng.permutation(len(x))
        losses = []
        for s in range(0, len(x), batch):
            b = order[s:s + batch]
            pred, cache = net.forward(x[b])
            net.step(net.backward(cache, y[b]), lr)
            losses.append(float(np.mean((pred - y[b]) ** 2)))
        if (ep + 1) % 20 == 0 or ep == 0:
            mae = np.abs(net.forward(x)[0] - y).mean(axis=0)
            row = {"epoch": ep + 1, "mse": round(float(np.mean(losses)), 6),
                   "mae_delta": round(float(mae[0]), 4), "mae_speed": round(float(mae[1]), 4)}
            hist.append(row)
            log("epoch {epoch:>3}/{epochs}  mse={mse:.5f}  MAE(δ)={mae_delta:.3f}  "
                "MAE(v)={mae_speed:.3f}".format(epochs=epochs, **row))
    return hist


def dagger(env: AckermannEnv, net: TrackerNet, rounds: int = 1, samples: int = 600,
           seed: int = 0, log=print):
    """DAgger：让网络自己跑，在它访问到的状态上问 MPPI 该怎么打。"""
    from .mppi import MPPIConfig, MPPIController
    from .nav import PoseEstimator
    from . import track
    from .net import build_features, DELTA_MAX, V_MIN, V_MAX

    path = env.path
    xs, ys = [], []
    for r in range(rounds):
        env.reset()
        est = PoseEstimator(env)
        mppi = MPPIController(path, MPPIConfig(), seed=1234 + r)
        delta_prev, v_prev = 0.0, 0.9
        for _ in range(samples):
            est.correct(path, env.lane_errors_from_lidar()[0])
            pose = est.pose
            idx = track.nearest_index(path, pose[:2], est.idx)
            e_est, psi_est = track.track_errors(path, pose[:2], pose[2], idx)
            feats = build_features(e_est, psi_est, env.state().v, delta_prev, v_prev,
                                   path, idx, pose)
            v_cmd, delta = net.act(feats)
            ex_v, ex_d = mppi.act(pose, v_now=env.state().v)
            xs.append(feats)
            ys.append([np.clip(ex_d / DELTA_MAX, -1, 1),
                       np.clip((ex_v - V_MIN) / (V_MAX - V_MIN), 0, 1)])
            delta_prev, v_prev = delta, v_cmd
            state_prev = env.state()
            new = env.step(v_cmd, delta)
            v_o, w_o = env.odom(state_prev, new)
            est.predict(v_o, w_o)
            if env.crashed or env.progress > 0.98:
                break
        log(f"DAgger 第 {r + 1} 轮：采集 {len(xs)} 条")
    return np.stack(xs).astype(np.float32), np.stack(ys).astype(np.float32)


def plot_training(hist, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ep = [h["epoch"] for h in hist]
    fig, ax1 = plt.subplots(figsize=(8, 4))
    ax1.plot(ep, [h["mse"] for h in hist], "o-", color="#2f80ed", label="train MSE")
    ax1.set_xlabel("epoch")
    ax1.set_ylabel("MSE", color="#2f80ed")
    ax2 = ax1.twinx()
    ax2.plot(ep, [h["mae_delta"] for h in hist], "s--", color="#27ae60", label="MAE δ")
    ax2.plot(ep, [h["mae_speed"] for h in hist], "^:", color="#eb5757", label="MAE v")
    ax2.set_ylabel("normalised MAE")
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [ln.get_label() for ln in lines], loc="upper right", fontsize=8)
    ax1.grid(alpha=0.3)
    fig.suptitle("distilling MPPI into a 12-64-32-2 MLP")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def save_gif(frames, path: Path, duration: int = 80):
    from PIL import Image, ImageDraw

    images = []
    for rgb, info in frames:
        img = Image.fromarray(rgb).resize((200, 200), Image.BILINEAR)
        canvas = Image.new("RGB", (200, 240), (18, 20, 26))
        canvas.paste(img, (0, 0))
        d = ImageDraw.Draw(canvas)
        e, psi, v, delta = info
        d.text((6, 206), f"e={e:+.2f} m  psi={psi:+.2f} rad", fill=(210, 220, 235))
        d.text((6, 222), f"v={v:.2f} m/s  delta={delta:+.2f} rad", fill=(210, 220, 235))
        images.append(canvas.convert("P", palette=Image.ADAPTIVE, colors=32))
    images[0].save(path, save_all=True, append_images=images[1:], duration=duration,
                   loop=0, optimize=True)
    return path.stat().st_size, len(images)


def save_report(net_cfg, hist, runs, agg, out_dir) -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    payload = {"net": net_cfg, "training": hist, "runs": runs, "aggregate": agg}
    (out / "metrics.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False),
                                      encoding="utf-8")
    return payload
