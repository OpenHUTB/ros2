"""专家数据生成 + 神经网络训练 + 出图。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .env import DiffDriveEnv
from .nav import NavConfig, make_expert_dataset
from .net import LocalPlannerNet


def train(env: DiffDriveEnv, n: int = 2000, epochs: int = 80, batch: int = 64,
          lr: float = 8e-4, seed: int = 0, log=print):
    """用 A*+纯追踪专家训练局部规划网络。

    `make_expert_dataset()` 会反复 `env.reset(...)` 采样位姿，训练结束把 env 的
    场景与起点原样还原（评审要求在 #689 里提过一次，这里一开始就做对）。
    """
    scene0, seed0 = env.scene, env.seed
    pose0, yaw0 = env.state().pos.copy(), env.state().yaw
    x, y = make_expert_dataset(env, n=n, seed=seed, cfg=NavConfig(), log=log)
    net = LocalPlannerNet(seed=seed)
    rng = np.random.default_rng(seed)
    history = []
    for ep in range(epochs):
        order = rng.permutation(len(x))
        losses = []
        for s in range(0, len(x), batch):
            idx = order[s:s + batch]
            pred, cache = net.forward(x[idx])
            net.step(net.backward(cache, y[idx]), lr)
            losses.append(float(np.mean((pred - y[idx]) ** 2)))
        if (ep + 1) % 10 == 0 or ep == 0:
            mae = np.abs(net.forward(x)[0] - y).mean(axis=0)
            row = {"epoch": ep + 1, "mse": round(float(np.mean(losses)), 6),
                   "mae_speed": round(float(mae[0]), 4), "mae_yaw": round(float(mae[1]), 4)}
            history.append(row)
            log("epoch {epoch:>3}/{epochs}  mse={mse:.5f}  MAE(v)={mae_speed:.3f}  "
                "MAE(w)={mae_yaw:.3f}".format(epochs=epochs, **row))
    env.reset(scene=scene0, seed=seed0, pos=pose0, yaw=yaw0)
    return net, history, (x, y)


def plot_training(history, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ep = [h["epoch"] for h in history]
    fig, ax1 = plt.subplots(figsize=(8, 4))
    ax1.plot(ep, [h["mse"] for h in history], "o-", color="#2f80ed", label="train MSE")
    ax1.set_xlabel("epoch")
    ax1.set_ylabel("train MSE", color="#2f80ed")
    ax1.tick_params(axis="y", labelcolor="#2f80ed")
    ax2 = ax1.twinx()
    ax2.plot(ep, [h["mae_speed"] for h in history], "s--", color="#27ae60", label="MAE speed")
    ax2.plot(ep, [h["mae_yaw"] for h in history], "^:", color="#eb5757", label="MAE yaw")
    ax2.set_ylabel("validation MAE (normalised)")
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [l.get_label() for l in lines], loc="upper right", fontsize=8)
    ax1.grid(alpha=0.3)
    fig.suptitle("local planner MLP training (numpy only)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def save_gif(frames, path: Path, duration: int = 90) -> tuple[int, int]:
    """帧格式：(chase_rgb, scan, slam_pose, dist, logodds, truth_xy) -> 三栏：3D / 雷达 / 地图。"""
    from PIL import Image, ImageDraw

    images = []
    for rgb, scan, pose, dist, logodds, truth_xy in frames:
        base = Image.fromarray(rgb).resize((200, 200), Image.BILINEAR)
        canvas = Image.new("RGB", (200 + 170 + 170, 226), (18, 20, 26))
        canvas.paste(base, (0, 24))
        draw = ImageDraw.Draw(canvas)
        # 雷达俯视图
        cx, cy, r = 250, 124, 78
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(70, 78, 92))
        for k, d in enumerate(scan):
            ang = 2.0 * np.pi * k / len(scan)
            rr = r * min(d, 6.0) / 6.0
            px, py = cx + rr * np.cos(ang), cy - rr * np.sin(ang)
            draw.point((px, py), fill=(235, 120, 90) if d < 5.99 else (60, 66, 78))
        # 第三栏：在线占据栅格（黑=未知，灰=空闲，白=占据）+ 车的位置
        # 白=空闲、黑=占据、灰=未知（与 ROS map_server 的观感一致）
        occ = np.clip((2.0 - logodds) / 4.0, 0, 1)
        img = Image.fromarray((occ[::-1] * 255).astype(np.uint8)).resize((160, 160), Image.BILINEAR)
        canvas.paste(img.convert("RGB"), (376, 44))
        mx = 376 + int((pose[0] - (-1.0)) / 14.0 * 160)
        my = 44 + int((7.0 - pose[1]) / 14.0 * 160)
        draw.ellipse([mx - 3, my - 3, mx + 3, my + 3], fill=(80, 170, 255))
        tx = 376 + int((truth_xy[0] + 1.0) / 14.0 * 160)
        ty = 44 + int((7.0 - truth_xy[1]) / 14.0 * 160)
        draw.point((tx, ty), fill=(240, 120, 90))

        draw.text((0, 6), "chase view", fill=(200, 210, 225))
        draw.text((206, 6), "lidar (ego frame)", fill=(200, 210, 225))
        draw.text((376, 6), "online SLAM map", fill=(200, 210, 225))
        draw.text((206, 206), "pose %.1f,%.1f  d=%.1f m" % (pose[0], pose[1], dist),
                  fill=(225, 230, 240))
        draw.text((376, 206), "blue=SLAM  red=truth", fill=(225, 230, 240))
        images.append(canvas)
    images[0].save(path, save_all=True, append_images=images[1:], duration=duration, loop=0)
    return path.stat().st_size, len(images)


def save_report(history, records, out_dir) -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    summary = {"training": history, "runs": records}
    (out / "metrics.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False),
                                      encoding="utf-8")
    return summary
