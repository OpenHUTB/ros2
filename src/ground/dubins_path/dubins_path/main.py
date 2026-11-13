# -*- coding: utf-8 -*-
"""Dubins 曲线轨迹生成与可视化。

支持交互展示与离屏录制两种模式：
  交互模式：实时显示 6 条候选路径、最短路径高亮、起末位姿箭头
  录制模式：离屏渲染并合成演示 gif
"""

from __future__ import annotations
import argparse
import math
import os
from typing import List, Tuple

import pygame

from . import config as C
from .dubins import (
    shortest_path,
    all_paths,
    sample_path,
    mod2pi,
)

Pose = Tuple[float, float, float]


def _font(size: int, bold: bool = False):
    """创建字体，SysFont 不可用时回退到 pygame 内置字体。

    pygame 2.6.1 + Python 3.12 在 Windows 上调用
    ``pygame.font.SysFont('monospace', 14)`` 会抛
    ``TypeError: expected str, bytes or os.PathLike object, not int``，
    这里统一兜底，避免 ``python main.py`` 直接崩。
    """
    try:
        return pygame.font.SysFont("monospace", size, bold=bold)
    except Exception:
        return pygame.font.Font(None, size + 4)


# ---------------------------------------------------------------------------
# 渲染
# ---------------------------------------------------------------------------
def draw_grid(surface):
    step = 30
    for x in range(0, C.WIDTH, step):
        pygame.draw.line(surface, C.GRID_COLOR, (x, 0), (x, C.HEIGHT))
    for y in range(0, C.HEIGHT, step):
        pygame.draw.line(surface, C.GRID_COLOR, (0, y), (C.WIDTH, y))


def draw_pose_arrow(surface, pose: Pose, color, label=None, font=None):
    """绘制位姿箭头（位置 + 朝向）。"""
    x, y, yaw = pose
    # 圆点
    pygame.draw.circle(surface, color, (int(x), int(y)), 8)
    pygame.draw.circle(surface, (255, 255, 255), (int(x), int(y)), 8, 1)
    # 朝向箭头
    arrow_len = 40
    ex = x + arrow_len * math.cos(yaw)
    ey = y + arrow_len * math.sin(yaw)
    pygame.draw.line(surface, color, (x, y), (ex, ey), 3)
    # 箭头头
    head = 10
    for sign in (-1, 1):
        ang = yaw + sign * math.pi / 6.0
        hx = ex - head * math.cos(ang)
        hy = ey - head * math.sin(ang)
        pygame.draw.line(surface, color, (ex, ey), (hx, hy), 2)
    if label and font:
        surface.blit(font.render(label, True, color), (int(x) + 12, int(y) + 12))


def draw_path(surface, samples: List[Pose], color, width=2):
    """绘制路径（采样点连线）。"""
    if len(samples) < 2:
        return
    pts = [(int(s[0]), int(s[1])) for s in samples]
    pygame.draw.lines(surface, color, False, pts, width)


def draw_hud(surface, font, start: Pose, end: Pose, radius: float,
             best_type: str, best_length: float, all_results):
    lines = [
        f"start: ({start[0]:.0f}, {start[1]:.0f}, {math.degrees(start[2]):.0f} deg)",
        f"end:   ({end[0]:.0f}, {end[1]:.0f}, {math.degrees(end[2]):.0f} deg)",
        f"radius: {radius:.0f} px",
        f"shortest: {best_type}  length = {best_length:.1f} px",
        f"candidates ({len(all_results)}):",
    ]
    for name, _, length in all_results:
        marker = " *" if name == best_type else ""
        lines.append(f"  {name}: {length:.1f}{marker}")
    y = 10
    for line in lines:
        text = font.render(line, True, C.TEXT_COLOR)
        surface.blit(text, (10, y))
        y += 18


# ---------------------------------------------------------------------------
# 录制
# ---------------------------------------------------------------------------
def record_gif(output_path: str, start: Pose, end: Pose, radius: float):
    """离屏渲染完整动画并合成 gif。"""
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.init()
    screen = pygame.display.set_mode((C.WIDTH, C.HEIGHT))
    font = _font(14)
    clock = pygame.time.Clock()

    best_type, best_tpq, best_length = shortest_path(start, end, radius)
    all_results = all_paths(start, end, radius)
    shortest_samples = sample_path(start, radius, best_tpq, best_type, C.CURVE_SAMPLES)

    frames = []
    total_steps = C.ANIM_STEPS
    # 阶段 1: 动画绘制最短路径
    for step in range(1, total_steps + 1):
        frac = step / total_steps
        n_now = max(2, int(C.CURVE_SAMPLES * frac))
        drawn = shortest_samples[:n_now]
        screen.fill(C.BG_COLOR)
        draw_grid(screen)
        # 淡显所有候选路径
        for idx, (name, tpq, _) in enumerate(all_results):
            if name == best_type:
                continue
            cand_samples = sample_path(start, radius, tpq, name, C.CURVE_SAMPLES)
            color = C.CANDIDATE_COLORS[idx % len(C.CANDIDATE_COLORS)]
            draw_path(screen, cand_samples, color, 1)
        # 高亮已绘制的最短路径段
        draw_path(screen, drawn, C.SHORTEST_COLOR, 3)
        draw_pose_arrow(surface=screen, pose=start, color=C.START_COLOR,
                        label="start", font=font)
        draw_pose_arrow(surface=screen, pose=end, color=C.END_COLOR,
                        label="end", font=font)
        draw_hud(screen, font, start, end, radius, best_type, best_length, all_results)
        pygame.display.flip()
        clock.tick(60)
        if step % C.RECORD_EVERY == 0:
            frames.append(_scale_frame(screen))

    # 阶段 2: 完整路径 + 尾部停留
    for _ in range(C.RECORD_TAIL):
        screen.fill(C.BG_COLOR)
        draw_grid(screen)
        for idx, (name, tpq, _) in enumerate(all_results):
            if name == best_type:
                continue
            cand_samples = sample_path(start, radius, tpq, name, C.CURVE_SAMPLES)
            color = C.CANDIDATE_COLORS[idx % len(C.CANDIDATE_COLORS)]
            draw_path(screen, cand_samples, color, 1)
        draw_path(screen, shortest_samples, C.SHORTEST_COLOR, 3)
        draw_pose_arrow(surface=screen, pose=start, color=C.START_COLOR,
                        label="start", font=font)
        draw_pose_arrow(surface=screen, pose=end, color=C.END_COLOR,
                        label="end", font=font)
        draw_hud(screen, font, start, end, radius, best_type, best_length, all_results)
        pygame.display.flip()
        clock.tick(30)
        frames.append(_scale_frame(screen))

    pygame.quit()
    _save_gif(frames, output_path)
    return best_type, best_length


def _scale_frame(screen):
    surf = pygame.transform.smoothscale(screen,
                                        (int(C.WIDTH * C.RECORD_SCALE),
                                         int(C.HEIGHT * C.RECORD_SCALE)))
    raw = pygame.image.tostring(surf, "RGB")
    from PIL import Image
    return Image.frombytes("RGB",
                           (int(C.WIDTH * C.RECORD_SCALE),
                            int(C.HEIGHT * C.RECORD_SCALE)), raw)


def _save_gif(frames, path):
    from PIL import Image
    if not frames:
        return
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=50, loop=0, disposal=2)
    print(f"[Dubins] gif saved: {path} ({len(frames)} frames)")


# ---------------------------------------------------------------------------
# 交互模式
# ---------------------------------------------------------------------------
def run_interactive(start: Pose, end: Pose, radius: float):
    pygame.init()
    screen = pygame.display.set_mode((C.WIDTH, C.HEIGHT))
    pygame.display.set_caption("Dubins Path")
    font = _font(14)
    clock = pygame.time.Clock()

    best_type, best_tpq, best_length = shortest_path(start, end, radius)
    all_results = all_paths(start, end, radius)
    shortest_samples = sample_path(start, radius, best_tpq, best_type, C.CURVE_SAMPLES)
    # 预计算所有候选路径采样
    cand_data = []
    for idx, (name, tpq, _) in enumerate(all_results):
        if name == best_type:
            cand_data.append((name, None, C.CANDIDATE_COLORS[idx % len(C.CANDIDATE_COLORS)]))
        else:
            samples = sample_path(start, radius, tpq, name, C.CURVE_SAMPLES)
            cand_data.append((name, samples, C.CANDIDATE_COLORS[idx % len(C.CANDIDATE_COLORS)]))

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        screen.fill(C.BG_COLOR)
        draw_grid(screen)
        # 候选路径
        for name, samples, color in cand_data:
            if samples is not None:
                draw_path(screen, samples, color, 1)
        # 最短路径高亮
        draw_path(screen, shortest_samples, C.SHORTEST_COLOR, 3)
        draw_pose_arrow(surface=screen, pose=start, color=C.START_COLOR,
                        label="start", font=font)
        draw_pose_arrow(surface=screen, pose=end, color=C.END_COLOR,
                        label="end", font=font)
        draw_hud(screen, font, start, end, radius, best_type, best_length, all_results)
        pygame.display.flip()
        clock.tick(C.FPS)
    pygame.quit()


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Dubins 曲线轨迹生成与可视化")
    parser.add_argument("--mode", choices=("interactive", "record"), default=None,
                        help="运行模式；ROS 2 launch 用这个参数驱动")
    parser.add_argument("--record", action="store_true",
                        help="录制模式：离屏渲染并合成演示 gif（等价 --mode record）")
    parser.add_argument("--no-display", action="store_true",
                        help="无显示设备环境（VM/CI）下离屏运行（等价 --mode record）")
    parser.add_argument("--output", default="dubins_demo.gif",
                        help="录制模式下输出 gif 路径")
    parser.add_argument("--start", default="",
                        help="起点位姿，格式 'x,y,yaw_deg'，留空用默认/params.yaml")
    parser.add_argument("--end", default="",
                        help="终点位姿，格式 'x,y,yaw_deg'，留空用默认/params.yaml")
    parser.add_argument("--radius", default="",
                        help="最小转弯半径（像素），留空用默认/params.yaml")
    parser.add_argument("--params", default="",
                        help="params.yaml 路径；留空则按 环境变量/安装空间/源码树 顺序自动查找")
    args = parser.parse_args()

    used = C.load_params(args.params or None)
    if used:
        print(f"[Dubins] 参数文件: {used}", flush=True)
    else:
        print("[Dubins] 未找到 params.yaml，使用内置默认值", flush=True)

    if args.start:
        parts = args.start.split(",")
        start = (float(parts[0]), float(parts[1]), math.radians(float(parts[2])))
    else:
        start = tuple(C.DEFAULT_START)
    if args.end:
        parts = args.end.split(",")
        end = (float(parts[0]), float(parts[1]), math.radians(float(parts[2])))
    else:
        end = tuple(C.DEFAULT_END)
    radius = C.DEFAULT_RADIUS if args.radius in ("", None) else float(args.radius)

    record = args.record or args.no_display or args.mode == "record"
    if args.mode == "interactive":
        record = False

    if record:
        out = os.path.abspath(args.output)
        best_type, length = record_gif(out, start, end, radius)
        print(f"[Dubins] shortest: {best_type}, length = {length:.1f} px")
    else:
        run_interactive(start, end, radius)
