"""7 自由度人形手臂的线框动画（画圆轨迹）。"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .kinematics import SerialChain

try:  # pragma: no cover
    from PIL import Image
except Exception:  # noqa: BLE001
    Image = None  # type: ignore[assignment]

JOINT_COLOR = "#1a202c"
LINK_COLOR = "#2b6cb0"
EE_COLOR = "#c53030"
PATH_COLOR = "#9ae6b4"
TRACE_COLOR = "#f6ad55"


def create_circle_gif(
    chain: SerialChain,
    solutions: np.ndarray,
    targets: np.ndarray,
    path: str | Path,
    plane: str = "xz",
    fps: int = 12,
    hold_last_s: float = 0.8,
) -> Path | None:
    """逐帧渲染关节链，末端随目标圆运动。"""
    if Image is None:
        print("[animate] 未安装 Pillow，跳过 GIF")
        return None
    if len(solutions) == 0:
        raise ValueError("solutions 不能为空")

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(5.6, 5.2), constrained_layout=True)
    frames = []
    trace: list[np.ndarray] = []
    for k, q in enumerate(solutions):
        trace.append(np.asarray(chain.forward(q).position, dtype=float))
        ax.clear()
        pts = chain.link_points(q)
        ax.plot(pts[:, 0], pts[:, 2], "-o", color=LINK_COLOR, lw=2.6,
                ms=4.5, mfc=JOINT_COLOR, mec=JOINT_COLOR, zorder=3)
        ax.plot(targets[:, 0], targets[:, 2], ":", color=PATH_COLOR, lw=2.0,
                zorder=1)
        if len(trace) > 1:
            tr = np.asarray(trace)
            ax.plot(tr[:, 0], tr[:, 2], color=TRACE_COLOR, lw=1.4, alpha=0.8,
                    zorder=2)
        ax.scatter([targets[k, 0]], [targets[k, 2]], s=90, marker="+",
                   color="#2f855a", zorder=5, label="target")
        ax.scatter([pts[-1, 0]], [pts[-1, 2]], s=55, color=EE_COLOR, zorder=6,
                   label="end effector")
        ax.scatter([0.0], [0.30], s=60, marker="s", color=JOINT_COLOR, zorder=4,
                   label="shoulder")
        ax.set_xlim(-0.55, 0.55)
        ax.set_ylim(-0.20, 0.55)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Z [m]")
        ax.set_title(
            f"7-DOF arm tracing circle ({plane})  k = {k + 1}/{len(solutions)}",
            fontsize=10,
        )
        ax.grid(alpha=0.25)
        ax.legend(loc="upper right", fontsize=7)
        fig.canvas.draw()
        rgba = np.asarray(fig.canvas.buffer_rgba())
        frames.append(Image.fromarray(rgba[..., :3].copy()))
    plt.close(fig)

    frames.extend([frames[-1]] * max(1, int(round(hold_last_s * fps))))
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=int(round(1000.0 / fps)), loop=0, optimize=True)
    print(f"[animate] {path.name}: {len(frames)} 帧, "
          f"{path.stat().st_size / 1024:.0f} KB")
    return path
