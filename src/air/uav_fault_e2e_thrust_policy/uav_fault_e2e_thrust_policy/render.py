"""出图：演示动图（深度图 + 高度 + 指令）与对比汇总图。

图上只写 ASCII：matplotlib 的默认字体没有中文字形，写中文会变成方框。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


def _pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _save_gif(frames, out_path: Path, fps: int = 10) -> None:
    from PIL import Image

    if not frames:
        raise RuntimeError("没有采到任何帧")
    imgs = [Image.fromarray(np.asarray(f, dtype=np.uint8)) for f in frames]
    stride = max(1, int(round(len(imgs) / 200)))
    imgs = imgs[::stride]
    imgs[0].save(
        out_path, save_all=True, append_images=imgs[1:],
        duration=int(1000 / max(fps, 1)), loop=0,
        optimize=True, disposal=2,
    )


def make_policy_gif(out_path, api=None, policy_module=None, spec=None, seed: int = 0,
                    fps: int = 10, arch: str = "cnn") -> Path:
    """策略飞一次的动图：左深度图、中高度曲线、右指令柱状图。"""
    from . import policy as P
    from .data import make_plan
    from .env import ThrustEnv

    plt = _pyplot()
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plan = make_plan(spec or {"kind": "healthy"})
    env = ThrustEnv(api, plan=plan, seed=seed)
    frames = []
    try:
        ob = env.reset(seed=seed, plan=plan)
        history = [ob["depth"][-1].copy()] * P.DEPTH_STACK
        ts, zs, cmds = [], [], []
        done = False
        while not done:
            if policy_module is None:
                act = env.expert_command()
            else:
                act = P.predict(policy_module, {
                    "depth": P.stack_depth(history, policy_module),
                    "proprio": ob["proprio"][None]})[0]
                act = np.clip(act, 0.0, 1.0)
            history.append(ob["depth"][-1].copy())
            ob, info = env.step(act)
            ts.append(info.t)
            zs.append(float(info.position[2]))
            cmds.append(info.actual.copy())
            done = info.done
            if len(ts) % 10 == 0:
                frames.append(_render_frame(plt, ts, zs, cmds, ob, info, plan, arch))
        frames.append(_render_frame(plt, ts, zs, cmds, ob, info, plan, arch))
    finally:
        env.close()
    _save_gif(frames, out_path, fps)
    return out_path


def _render_frame(plt, ts, zs, cmds, ob, info, plan, arch):
    fig = plt.figure(figsize=(9.0, 2.9), dpi=80)
    ax0 = fig.add_subplot(1, 3, 1)
    ax1 = fig.add_subplot(1, 3, 2)
    ax2 = fig.add_subplot(1, 3, 3)

    ax0.imshow(ob["depth"][0], cmap="viridis", vmin=0.0, vmax=1.0)
    ax0.set_title("depth (front camera)", fontsize=8)
    ax0.set_xticks([])
    ax0.set_yticks([])

    ax1.plot(ts, zs, color="#4C72B0", lw=1.2)
    ax1.axhline(1.5, color="#999999", ls="--", lw=0.8)
    ax1.set_ylim(0.0, 3.0)
    ax1.set_xlim(max(0.0, (ts[-1] if ts else 0.0) - 6.0), max(6.0, ts[-1] if ts else 1.0))
    ax1.set_title("altitude (m)", fontsize=8)
    ax1.tick_params(labelsize=7)

    ax2.bar(["r0", "r1", "r2", "r3"], cmds[-1], color="#C44E52")
    ax2.set_ylim(0.0, 6.0)
    ax2.set_title("actual rotor thrust (N)", fontsize=8)
    ax2.tick_params(labelsize=7)

    gains = info.gains if info is not None else np.ones(4)
    fig.suptitle(
        f"{arch}  t={ts[-1]:.2f}s  x={info.position[0]:+.2f}m  "
        f"gains={np.round(gains, 2).tolist()}  {plan.kind}",
        fontsize=8)
    fig.tight_layout()
    fig.canvas.draw()
    frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return frame


def make_summary_png(summary: dict, out_path) -> Path:
    """评估汇总：成功率与指令偏离。"""
    plt = _pyplot()
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    modes = list(summary.keys())
    colors = {"expert": "#4C72B0", "mlp": "#8172B3", "cnn": "#55A868",
              "cnn_stack": "#C44E52"}
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.0), dpi=90)
    rates = [summary[m]["success_rate"] for m in modes]
    axes[0].bar(modes, rates, color=[colors.get(m, "#4C72B0") for m in modes])
    axes[0].set_ylim(0.0, 1.05)
    axes[0].set_ylabel("success rate")
    n = summary[modes[0]]["n"] if modes else 0
    axes[0].set_title(f"success rate ({n} flights)")
    devs = [summary[m]["mean_cmd_dev"] for m in modes]
    axes[1].bar(modes, devs, color=[colors.get(m, "#4C72B0") for m in modes])
    axes[1].set_ylabel("mean |cmd - expert|")
    axes[1].set_title("deviation from expert command")
    for ax in axes:
        ax.tick_params(labelsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    return out_path
