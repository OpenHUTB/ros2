"""行走动图渲染：侧视火柴人 + 支撑区间 + ZMP。

用 matplotlib 的 Agg 后端逐帧画图，再用 Pillow 合成 GIF。
Pillow 不是必需依赖：没装时函数返回 None，主程序只跳过 GIF。
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .footstep import FootstepPlan
from .gait import GaitResult

try:  # pragma: no cover - 仅在缺依赖的分支上生效
    from PIL import Image
except Exception:  # noqa: BLE001 - 任何导入失败都按“没有 Pillow”处理
    Image = None  # type: ignore[assignment]

STANCE_COLOR = "#1a365d"   # 支撑腿：深蓝
SWING_COLOR = "#90cdf4"    # 摆动腿：浅蓝
SUPPORT_COLOR = "#2f855a"
ZMP_COLOR = "#c53030"
DARK = "#1a202c"
GRAY = "#4a5568"

HIP_Z = 0.85          # 髋关节高度 [m]
THIGH = 0.45          # 大腿长 [m]
SHANK = 0.45          # 小腿长 [m]
TORSO = 0.55          # 髋到肩的躯干长度 [m]


def _knee_2link(hip_x: float, hip_z: float, foot_x: float, foot_z: float):
    """两连杆逆解，返回 (膝 x, 膝 z, 脚 x, 脚 z)。

    膝关节向身体后方（-x）弯，符合人腿的屈膝方向；脚超出可达范围时
    沿同一方向缩短到可达边界。
    """
    dx = foot_x - hip_x
    dz = foot_z - hip_z
    dist = float(np.hypot(dx, dz))
    reach = THIGH + SHANK
    if dist < 1e-9:
        return hip_x, hip_z - 0.5 * reach, hip_x, hip_z - reach
    if dist > reach * 0.999:
        scale = 0.999 * reach / dist
        foot_x = hip_x + dx * scale
        foot_z = hip_z + dz * scale
        dx, dz = foot_x - hip_x, foot_z - hip_z
        dist = reach * 0.999

    cos_a = (THIGH**2 + dist**2 - SHANK**2) / (2.0 * THIGH * dist)
    cos_a = float(np.clip(cos_a, -1.0, 1.0))
    sin_a = float(np.sqrt(max(0.0, 1.0 - cos_a**2)))
    ux, uz = dx / dist, dz / dist
    # 髋-脚单位向量逆时针旋转 90° 得到 (uz, -ux)；取 x 分量为负的那侧。
    nx, nz = uz, -ux
    if nx > 0.0:
        nx, nz = -nx, -nz
    knee_x = hip_x + THIGH * (cos_a * ux + sin_a * nx)
    knee_z = hip_z + THIGH * (cos_a * uz + sin_a * nz)
    return knee_x, knee_z, foot_x, foot_z


def _draw_frame(ax: plt.Axes, plan: FootstepPlan, result: GaitResult, k: int) -> None:
    t = float(result.time[k])
    com = float(result.x[k])
    ax.clear()

    lo, hi = float(result.support_left[k]), float(result.support_right[k])
    ax.plot([lo, hi], [0.0, 0.0], color=SUPPORT_COLOR, lw=7.0,
            solid_capstyle="butt", zorder=1, label="support polygon")

    for side in ("L", "R"):
        fx, fz, stance = plan.foot_pose_by_side(side, t)
        color = STANCE_COLOR if stance else SWING_COLOR
        ax.plot([fx - 0.075, fx + 0.075], [fz, fz], color=color, lw=5.0, zorder=3)
        kx, kz, ax_x, az_z = _knee_2link(com, HIP_Z, fx, fz)
        ax.plot([com, kx, ax_x], [HIP_Z, kz, az_z], color=color, lw=4.5, zorder=2)
        ax.scatter([kx], [kz], s=18, color=DARK, zorder=4)

    ax.plot([com, com], [HIP_Z, HIP_Z + TORSO], color=DARK, lw=6.0, zorder=3)
    ax.scatter([com], [HIP_Z + TORSO], s=70, color=DARK, zorder=5)
    ax.scatter([com], [HIP_Z], s=36, color=GRAY, zorder=5)

    zmp = float(result.zmp[k])
    ax.plot([zmp, zmp], [-0.02, 0.17], color=ZMP_COLOR, lw=2.5, zorder=6)
    ax.scatter([zmp], [0.0], s=34, color=ZMP_COLOR, zorder=7)

    ax.set_xlim(com - 0.85, com + 0.85)
    ax.set_ylim(-0.16, 1.60)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.set_title(
        f"t = {t:5.2f} s   phase = {result.phase[k]:6s}   "
        f"ZMP = {zmp:+.3f} m   margin = {float(result.margin[k]):.3f} m",
        fontsize=10,
    )
    ax.grid(alpha=0.25)


def create_walking_gif(
    result: GaitResult,
    plan: FootstepPlan,
    path: str | Path,
    fps: int = 20,
    max_frames: int = 120,
    hold_last_s: float = 0.8,
) -> Path | None:
    """把整段步态渲染成 GIF；Pillow 缺失时返回 None。"""
    if Image is None:
        print("[animate] 未安装 Pillow，跳过 GIF（PNG 不受影响）")
        return None

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    stride = max(1, int(np.ceil(len(result.time) / max_frames)))
    frame_idx = list(range(0, len(result.time), stride))
    if frame_idx[-1] != len(result.time) - 1:
        frame_idx.append(len(result.time) - 1)

    fig, ax = plt.subplots(figsize=(7.2, 4.6), constrained_layout=True)
    frames = []
    for k in frame_idx:
        _draw_frame(ax, plan, result, k)
        fig.canvas.draw()
        rgba = np.asarray(fig.canvas.buffer_rgba())
        frames.append(Image.fromarray(rgba[..., :3].copy()))
    plt.close(fig)

    frames.extend([frames[-1]] * max(1, int(round(hold_last_s * fps))))

    frames[0].save(
        path,
        save_all=True,
        append_images=frames[1:],
        duration=int(round(1000.0 / fps)),
        loop=0,
        optimize=True,
    )
    size_mb = path.stat().st_size / (1024.0 * 1024.0)
    print(f"[animate] GIF: {path.name} ({size_mb:.2f} MB, {len(frames)} frames)")
    return path
