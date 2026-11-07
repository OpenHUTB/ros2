# -*- coding: utf-8 -*-
"""Bezier 曲线轨迹生成与可视化。

支持交互展示与离屏录制两种模式：
  交互模式：实时显示 de Casteljau 算法动画、B 样条对比、曲率分布
  录制模式：离屏渲染并合成演示 gif
"""

from __future__ import annotations
import argparse
import os
import math
import io
from typing import List, Tuple

import numpy as np
import pygame

from . import config as C
from .bezier import (
    bezier_point_de_casteljau,
    bezier_point_bernstein,
    bezier_derivative,
    curvature,
    arc_length,
    sample_curve,
    sample_bspline,
    binomial,
)

Point = Tuple[float, float]


def build_control_points() -> List[Point]:
    return [tuple(p) for p in C.DEFAULT_CONTROL_POINTS]


def build_bspline_points() -> List[Point]:
    return [tuple(p) for p in C.BSPLINE_CONTROL_POINTS]


# ---------------------------------------------------------------------------
# 渲染
# ---------------------------------------------------------------------------
def draw_grid(surface):
    step = 30
    for x in range(0, C.WIDTH, step):
        pygame.draw.line(surface, C.GRID_COLOR, (x, 0), (x, C.HEIGHT))
    for y in range(0, C.HEIGHT, step):
        pygame.draw.line(surface, C.GRID_COLOR, (0, y), (C.WIDTH, y))


def draw_control_polygon(surface, points: List[Point]):
    if len(points) < 2:
        return
    for i in range(len(points) - 1):
        pygame.draw.line(surface, C.CONTROL_POLY_COLOR,
                         points[i], points[i + 1], 1)


def draw_curve(surface, samples: List[Point], color, width=2):
    if len(samples) < 2:
        return
    pygame.draw.lines(surface, color, False, samples, width)


def draw_control_points(surface, points: List[Point]):
    for i, (x, y) in enumerate(points):
        color = C.ENDPOINT_COLOR if i in (0, len(points) - 1) else C.CONTROL_POINT_COLOR
        pygame.draw.circle(surface, color, (int(x), int(y)), 7)
        pygame.draw.circle(surface, (255, 255, 255), (int(x), int(y)), 7, 1)


def draw_de_casteljau(surface, points: List[Point], t: float):
    """绘制 de Casteljau 算法的中间结构（各级插值线段与点）。"""
    levels = [points]
    current = points
    for level in range(1, len(points)):
        next_level = []
        for i in range(len(current) - 1):
            x = (1 - t) * current[i][0] + t * current[i + 1][0]
            y = (1 - t) * current[i][1] + t * current[i + 1][1]
            next_level.append((x, y))
        levels.append(next_level)
        current = next_level

    # 绘制各级线段（颜色渐暗）
    for lvl, pts in enumerate(levels[1:], 1):
        alpha = max(30, 120 - lvl * 20)
        color = (alpha + 60, alpha + 80, alpha + 100)
        if len(pts) >= 2:
            pygame.draw.lines(surface, color, False, pts, 1)
        for p in pts:
            pygame.draw.circle(surface, color, (int(p[0]), int(p[1])), 4)

    # 最终点
    if levels[-1]:
        p = levels[-1][0]
        pygame.draw.circle(surface, C.CURVE_COLOR, (int(p[0]), int(p[1])), 6)
        pygame.draw.circle(surface, (255, 255, 255), (int(p[0]), int(p[1])), 6, 1)


def draw_curvature_strip(surface, points: List[Point], x_offset=10, y_offset=120):
    """在左侧绘制曲率分布条形图。"""
    samples = C.CURVATURE_SAMPLES
    w = 180
    h = 50
    bg_rect = pygame.Rect(x_offset, y_offset, w, h)
    pygame.draw.rect(surface, (20, 24, 34), bg_rect)
    pygame.draw.rect(surface, C.GRID_COLOR, bg_rect, 1)

    kappas = []
    for i in range(samples):
        t = i / max(samples - 1, 1)
        kappas.append(curvature(points, t))
    max_k = max(kappas) if kappas else 1.0
    if max_k < 1e-9:
        max_k = 1.0

    bar_w = w / samples
    for i, k in enumerate(kappas):
        bar_h = int(h * k / max_k)
        bx = x_offset + int(i * bar_w)
        by = y_offset + h - bar_h
        pygame.draw.rect(surface, C.CURVATURE_COLOR,
                         (bx, by, max(1, int(bar_w)), bar_h))
    # 标签
    font_sm = pygame.font.SysFont("monospace", 11)
    surface.blit(font_sm.render("curvature distribution", True, C.HUD_COLOR),
                 (x_offset, y_offset - 15))


def draw_hud(surface, font, points: List[Point], t: float, anim_step: int,
             total_steps: int, length: float, max_curv: float):
    lines = [
        f"order: {len(points) - 1}  (control pts: {len(points)})",
        f"t = {t:.3f}   step {anim_step}/{total_steps}",
        f"arc length: {length:.1f} px",
        f"max curvature: {max_curv:.6f}",
        f"binomial C({len(points)-1},2) = {binomial(len(points)-1, 2)}",
    ]
    y = 10
    for line in lines:
        text = font.render(line, True, C.TEXT_COLOR)
        surface.blit(text, (10, y))
        y += 18
    # 曲率条形图（放在 HUD 文字下方，不遮挡）
    draw_curvature_strip(surface, points)


# ---------------------------------------------------------------------------
# 录制
# ---------------------------------------------------------------------------
def record_gif(output_path: str, points: List[Point], bspline_pts: List[Point]):
    """离屏渲染完整动画并合成 gif。"""
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.init()
    screen = pygame.display.set_mode((C.WIDTH, C.HEIGHT))
    font = pygame.font.SysFont("monospace", 14)
    clock = pygame.time.Clock()

    total_steps = C.ANIM_STEPS
    curve_samples = sample_curve(points, C.CURVE_SAMPLES)
    bspline_samples = sample_bspline(bspline_pts, degree=3, n=C.CURVE_SAMPLES)
    length = arc_length(points)
    max_curv = max(curvature(points, i / 100) for i in range(101))

    frames = []
    # 阶段 1: de Casteljau 动画（曲线逐步绘制）
    for step in range(1, total_steps + 1):
        t = step / total_steps
        screen.fill(C.BG_COLOR)
        draw_grid(screen)
        draw_control_polygon(screen, points)
        # 已绘制的曲线部分
        drawn = [bezier_point_de_casteljau(points, i / total_steps * t)
                 for i in range(int(t * C.CURVE_SAMPLES) + 1)]
        if len(drawn) >= 2:
            draw_curve(screen, drawn, C.CURVE_COLOR, 2)
        draw_de_casteljau(screen, points, t)
        draw_control_points(screen, points)
        draw_hud(screen, font, points, t, step, total_steps, length, max_curv)
        pygame.display.flip()
        clock.tick(60)
        if step % C.RECORD_EVERY == 0:
            frames.append(_scale_frame(screen))

    # 阶段 2: 完整曲线 + B 样条对比
    for _ in range(C.RECORD_TAIL):
        screen.fill(C.BG_COLOR)
        draw_grid(screen)
        draw_control_polygon(screen, points)
        draw_curve(screen, curve_samples, C.CURVE_COLOR, 2)
        draw_curve(screen, bspline_samples, C.BSPLINE_COLOR, 2)
        draw_control_points(screen, points)
        # 图例
        font_sm = pygame.font.SysFont("monospace", 12)
        screen.blit(font_sm.render("Bezier", True, C.CURVE_COLOR), (C.WIDTH - 120, 10))
        screen.blit(font_sm.render("B-spline", True, C.BSPLINE_COLOR), (C.WIDTH - 120, 28))
        draw_hud(screen, font, points, 1.0, total_steps, total_steps, length, max_curv)
        pygame.display.flip()
        clock.tick(30)
        frames.append(_scale_frame(screen))

    pygame.quit()
    _save_gif(frames, output_path)
    return length, max_curv


def _scale_frame(screen):
    """缩放帧以控制 gif 体积。"""
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
    print(f"[Bezier] gif 已保存: {path} ({len(frames)} 帧)")


# ---------------------------------------------------------------------------
# 交互模式
# ---------------------------------------------------------------------------
def run_interactive(points: List[Point], bspline_pts: List[Point]):
    pygame.init()
    screen = pygame.display.set_mode((C.WIDTH, C.HEIGHT))
    pygame.display.set_caption("Bezier Curve Trajectory")
    font = pygame.font.SysFont("monospace", 14)
    clock = pygame.time.Clock()

    curve_samples = sample_curve(points, C.CURVE_SAMPLES)
    bspline_samples = sample_bspline(bspline_pts, degree=3, n=C.CURVE_SAMPLES)
    length = arc_length(points)
    max_curv = max(curvature(points, i / 100) for i in range(101))

    t = 0.0
    t_dir = 1
    t_speed = 0.008
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        t += t_speed * t_dir
        if t >= 1.0:
            t = 1.0
            t_dir = -1
        elif t <= 0.0:
            t = 0.0
            t_dir = 1

        screen.fill(C.BG_COLOR)
        draw_grid(screen)
        draw_control_polygon(screen, points)
        draw_curve(screen, curve_samples, C.CURVE_COLOR, 2)
        draw_curve(screen, bspline_samples, C.BSPLINE_COLOR, 2)
        draw_de_casteljau(screen, points, t)
        draw_control_points(screen, points)
        # 图例
        font_sm = pygame.font.SysFont("monospace", 12)
        screen.blit(font_sm.render("Bezier", True, C.CURVE_COLOR), (C.WIDTH - 120, 10))
        screen.blit(font_sm.render("B-spline", True, C.BSPLINE_COLOR), (C.WIDTH - 120, 28))
        draw_hud(screen, font, points, t, int(t * C.ANIM_STEPS), C.ANIM_STEPS, length, max_curv)
        pygame.display.flip()
        clock.tick(C.FPS)
    pygame.quit()


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Bezier 曲线轨迹生成与可视化")
    parser.add_argument("--record", action="store_true",
                        help="录制模式：离屏渲染并合成演示 gif")
    parser.add_argument("--no-display", action="store_true",
                        help="无显示设备环境（VM/CI）下离屏运行")
    parser.add_argument("--output", default="bezier_demo.gif",
                        help="录制模式下输出 gif 路径")
    args = parser.parse_args()

    points = build_control_points()
    bspline_pts = build_bspline_points()

    if args.record or args.no_display:
        out = os.path.abspath(args.output)
        length, max_curv = record_gif(out, points, bspline_pts)
        print(f"[Bezier] 弧长 {length:.1f} px, 最大曲率 {max_curv:.6f}")
    else:
        run_interactive(points, bspline_pts)
