#!/usr/bin/env python3
"""离线仿真：在三种地图上规划、平滑、时间参数化并出图。

无需 ROS / Gazebo，仅 NumPy + matplotlib。示例：

    python3 sim/run_sim.py --map corridor --out results/corridor.png
    python3 sim/run_sim.py --map maze --no-smooth
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


_CJK_FONT_FILES = (
    # Linux
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    # macOS
    "/System/Library/Fonts/PingFang.ttc",
    "/System/Library/Fonts/Hiragino Sans GB.ttc",
    "/Library/Fonts/Arial Unicode.ttf",
    # Windows
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/msyh.ttf",
    "C:/Windows/Fonts/simhei.ttf",
    "C:/Windows/Fonts/simsun.ttc",
    "C:/Windows/Fonts/Deng.ttf",
)

_CJK_FONT_FAMILIES = (
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",
    "Source Han Sans SC",
    "WenQuanYi Micro Hei",
    "WenQuanYi Zen Hei",
    "PingFang SC",
    "Hiragino Sans GB",
    "Microsoft YaHei",
    "SimHei",
    "SimSun",
    "DengXian",
    "Arial Unicode MS",
)


def _has_cjk_glyphs(path: str) -> bool:
    """字体文件里是否真有中文字形（只看文件名不够，DejaVu 也会被误选）。"""
    try:
        from matplotlib.ft2font import FT2Font

        return FT2Font(path).get_char_index(ord("路")) != 0
    except Exception:  # noqa: BLE001 - 字体损坏时按不可用处理
        return False


def setup_chinese_font() -> None:
    """让图里的中文正常显示：按 Linux / macOS / Windows 依次探测可用中文字体。

    找不到时明确告警（而不是静默退回 DejaVu Sans 导致图里全是方框）。
    也可用环境变量 AHP_CJK_FONT 指定字体文件路径。
    """
    candidates = []
    env_font = os.environ.get("AHP_CJK_FONT", "").strip()
    if env_font:
        candidates.append(env_font)
    candidates.extend(_CJK_FONT_FILES)

    for path in candidates:
        if os.path.exists(path) and _has_cjk_glyphs(path):
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
            plt.rcParams["font.family"] = name
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return

    for family in _CJK_FONT_FAMILIES:
        try:
            path = font_manager.findfont(
                font_manager.FontProperties(family=family), fallback_to_default=False
            )
        except Exception:  # noqa: BLE001 - 该字体未安装
            continue
        if os.path.exists(path) and _has_cjk_glyphs(path):
            plt.rcParams["font.family"] = family
            plt.rcParams["font.sans-serif"] = [family, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return

    print(
        "警告：未找到可用的中文字体，图中的中文会显示为方框。\n"
        "      Linux: sudo apt install fonts-noto-cjk 或 fonts-wqy-microhei\n"
        "      或设置环境变量 AHP_CJK_FONT=/path/to/font.ttf 手动指定字体。",
        file=sys.stderr,
    )
    plt.rcParams["font.family"] = "DejaVu Sans"


setup_chinese_font()

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.dirname(__file__))

from ros2_astar_planner.astar import AStarPlanner, path_length  # noqa: E402
from ros2_astar_planner.gridmap import (  # noqa: E402
    corridor_map,
    maze_map,
    random_obstacle_map,
)
from ros2_astar_planner.safety import (  # noqa: E402
    choose_connected_endpoints,
    inflation_cells,
)
from ros2_astar_planner.smoothing import (  # noqa: E402
    polyline_length,
    segment_is_free,
    smooth_path,
)
from ros2_astar_planner.trajectory import (  # noqa: E402
    check_acceleration,
    time_parameterize,
)

MAP_BUILDERS = {
    "corridor": lambda: corridor_map(),
    "maze": lambda: maze_map(),
    "random": lambda: random_obstacle_map(61, 41, 0.1, 0.08, seed=7),
}

# 各场景的起终点（栅格索引），避开已知障碍
ENDPOINTS = {
    "corridor": ((3, 3), (37, 58)),
    "maze": ((3, 3), (37, 58)),
    "random": ((3, 3), (37, 58)),
}

ROBOT_RADIUS = 0.20  # m
V_MAX, A_MAX, A_LAT_MAX = 0.6, 0.5, 1.0


def build_scene(name: str) -> dict:
    """构造场景：原图 -> 膨胀图 -> 端点修复 -> 规划 -> 平滑 -> 时间参数化。"""
    raw = MAP_BUILDERS[name]()
    inflate_n = inflation_cells(ROBOT_RADIUS, raw.spec.resolution)
    gm = raw.inflate(inflate_n)
    start0, goal0 = ENDPOINTS[name]
    start, goal, notes = choose_connected_endpoints(gm, start0, goal0)

    planner = AStarPlanner()
    result = planner.plan(gm, start, goal)
    if not result.success:
        return {"success": False, "reason": result.reason, "raw": raw, "gm": gm}

    world = np.asarray([gm.to_world(i, j) for i, j in result.cells], dtype=float)
    smoothed, metrics = smooth_path(gm, world, use_gradient=True, seed=3)
    traj = time_parameterize(smoothed, V_MAX, A_MAX, A_LAT_MAX)

    return {
        "success": True,
        "raw": raw,
        "gm": gm,
        "start": start,
        "goal": goal,
        "result": result,
        "world": world,
        "smoothed": smoothed,
        "metrics": metrics,
        "traj": traj,
        "fixes": notes,
        "accel_ok": check_acceleration(traj, A_MAX),
        "raw_grid_len": path_length(result.cells) * raw.spec.resolution,
    }


def draw(scene: dict, name: str, out: str, use_smooth: bool = True) -> None:
    """绘制 2x2 面板：地图与路径 / 速度剖面 / 曲率 / 指标。"""
    if not scene["success"]:
        print(f"[{name}] 规划失败：{scene['reason']}")
        return

    raw = scene["raw"]
    gm = scene["gm"]
    res = raw.spec.resolution
    h, w = raw.shape
    extent = (
        raw.spec.origin_x,
        raw.spec.origin_x + w * res,
        raw.spec.origin_y,
        raw.spec.origin_y + h * res,
    )

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    ax = axes[0, 0]
    ax.imshow(
        raw.grid,
        cmap="gray_r",
        origin="lower",
        extent=extent,
        interpolation="nearest",
        alpha=0.85,
    )
    # 膨胀层用浅红叠加显示
    inflated_only = (gm.grid != 0) & (raw.grid == 0)
    inflated_rgba = np.zeros((*inflated_only.shape, 4))
    inflated_rgba[inflated_only] = (1.0, 0.55, 0.55, 0.35)
    ax.imshow(inflated_rgba, origin="lower", extent=extent, interpolation="nearest")

    wp = scene["world"]
    ax.plot(wp[:, 0], wp[:, 1], "-", color="#1f77b4", lw=1.6, marker="o",
            ms=2.0, label="A* 原始路径")
    if use_smooth:
        sp = scene["smoothed"]
        ax.plot(sp[:, 0], sp[:, 1], "-", color="#d62728", lw=2.2,
                label="捷径+梯度平滑")
    si, sj = scene["start"]
    gi, gj = scene["goal"]
    sx, sy = gm.to_world(si, sj)
    gx, gy = gm.to_world(gi, gj)
    ax.plot(sx, sy, "o", color="#2ca02c", ms=11, label="起点")
    ax.plot(gx, gy, "*", color="#ff7f0e", ms=16, label="终点")
    ax.set_title(f"{name}：栅格 A* 路径（浅红=机器人半径膨胀层）")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_aspect("equal")

    traj = scene["traj"]
    ax2 = axes[0, 1]
    ax2.plot(traj.s, traj.v, color="#d62728", lw=2.0)
    ax2.axhline(V_MAX, ls="--", color="gray", lw=1.0, label=f"v_max={V_MAX}")
    ax2.set_title("梯形速度剖面 v(s)")
    ax2.set_xlabel("弧长 s (m)")
    ax2.set_ylabel("速度 (m/s)")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    ax3 = axes[1, 0]
    ax3.plot(traj.s, traj.curvature, color="#9467bd", lw=1.8)
    ax3.set_title("路径曲率 κ(s)")
    ax3.set_xlabel("弧长 s (m)")
    ax3.set_ylabel("曲率 (1/m)")
    ax3.grid(alpha=0.3)

    ax4 = axes[1, 1]
    ax4.axis("off")
    m = scene["metrics"]
    lines = [
        f"地图：{name}  {w}×{h} 格，分辨率 {res} m",
        f"膨胀：{inflation_cells(ROBOT_RADIUS, res)} 格（机器人半径 {ROBOT_RADIUS} m）",
        "",
        f"A* 扩展节点：{scene['result'].expansions}",
        f"A* 栅格代价：{scene['result'].cost:.2f}",
        f"原始栅格路径长度：{scene['raw_grid_len']:.2f} m",
        f"捷径后长度：{m['after_shortcut_length']:.2f} m",
        f"最终平滑长度：{m['final_length']:.2f} m",
        f"长度缩短：{(1 - m['final_length'] / max(m['raw_length'], 1e-9)) * 100:.1f}%",
        f"折点数：{m['raw_points']} → {m['final_points']}",
        f"转角数：{m['raw_turns']} → {m['final_turns']}",
        f"平滑后逐段无碰撞：{'是' if m['safe'] else '否'}",
        "",
        f"轨迹总长：{traj.total_length:.2f} m",
        f"总耗时：{traj.total_time:.2f} s",
        f"峰值速度：{traj.v_max_actual:.3f} m/s",
        f"加速度约束满足：{'是' if scene['accel_ok'] else '否'}",
    ]
    ax4.text(0.02, 0.98, "\n".join(lines), va="top", ha="left",
             fontsize=10, transform=ax4.transAxes)
    ax4.set_title("量化指标")

    fig.tight_layout()
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"[{name}] 已出图 {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description="A* 全局规划离线仿真")
    parser.add_argument("--map", choices=list(MAP_BUILDERS), default="corridor")
    parser.add_argument("--out", default="")
    parser.add_argument("--no-smooth", action="store_true", help="只画原始路径作对照")
    parser.add_argument("--json", action="store_true", help="打印指标 JSON")
    args = parser.parse_args()

    scene = build_scene(args.map)
    out = args.out or os.path.join(
        os.path.dirname(__file__), "..", "results", f"{args.map}.png"
    )
    draw(scene, args.map, out, use_smooth=not args.no_smooth)
    if args.json:
        payload = {
            "map": args.map,
            "success": scene["success"],
            "metrics": scene.get("metrics"),
            "expansions": scene["result"].expansions if scene["success"] else None,
            "trajectory_time": scene["traj"].total_time if scene["success"] else None,
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
