#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键实验入口（离线可运行，不依赖 ROS）。

用法:
    python main.py --shots      # 离屏运行闭环仿真，保存全过程截图
    python main.py --demo       # 打开交互窗口观察前沿探索过程

截图输出到仓库 docs/img/ 目录，文件名前缀与功能包一致。
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from uav_frontier_exploration_occupancy_nn.explorer import FrontierExplorer  # noqa: E402
from uav_frontier_exploration_occupancy_nn.occupancy_map import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN)

REPO_ROOT = os.path.abspath(os.path.join(_PKG_DIR, "..", "..", ".."))
IMG_DIR = os.path.join(REPO_ROOT, "docs", "img")

RECORD_EVERY = 6
GRID_W, GRID_H = 40, 30
RES = 0.25
X_MAX, Y_MAX = GRID_W * RES, GRID_H * RES


class Simulator:
    """前沿探索仿真器，封装闭环并记录栅格快照与关键指标。"""

    def __init__(self) -> None:
        self.ex = FrontierExplorer()
        self.history: list[dict] = []
        self.metrics = {"free": 0, "unknown": 0, "frontiers": 0,
                        "blocked": 0, "steps": 0, "max_x": 0.0, "max_y": 0.0}

    def step(self) -> dict:
        status = self.ex.step()
        self.metrics["free"] = status["free"]
        self.metrics["unknown"] = status["unknown"]
        self.metrics["frontiers"] = status["frontiers"]
        self.metrics["blocked"] = status["blocked_steps"]
        self.metrics["steps"] = status["steps"]
        self.metrics["max_x"] = max(self.metrics["max_x"], status["x"])
        self.metrics["max_y"] = max(self.metrics["max_y"], status["y"])
        if status["steps"] % RECORD_EVERY == 0:
            self.history.append(self._snapshot(status))
        return status

    def run(self, max_steps: int) -> None:
        for _ in range(max_steps):
            if self.ex.done:
                break
            self.step()

    def _snapshot(self, status: dict) -> dict:
        return {
            "grid": self.ex.map.grid.copy(),
            "trail": list(self.ex.trajectory),
            "target": None if status["target"] is None else dict(status["target"]),
            "scores": status["scores"].copy(),
            "status": dict(status),
        }

    def finalize(self) -> dict:
        return self.metrics


def _load_font(size: int):
    import pygame
    for path in (r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\msyh.ttc"):
        if os.path.exists(path):
            return pygame.font.Font(path, size)
    return pygame.font.Font(None, size)


def _cell_rect(c: int, r: int, pad: float, cw: float, ch: float):
    return pad + c * cw, pad + r * ch, cw, ch


def _draw_grid(surface, snap: dict, w: int, h: int) -> None:
    import pygame
    grid = snap["grid"]
    pad = 40.0
    cw = (w - 2 * pad) / GRID_W
    ch = (h - 2 * pad) / GRID_H
    frontier = set()
    for r in range(GRID_H):
        for c in range(GRID_W):
            if grid[r, c] == FREE:
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nc, nr = c + dc, r + dr
                        if 0 <= nc < GRID_W and 0 <= nr < GRID_H and grid[nr, nc] == UNKNOWN:
                            frontier.add((c, r))
    for r in range(GRID_H):
        for c in range(GRID_W):
            x, y, cw_, ch_ = _cell_rect(c, r, pad, cw, ch)
            if grid[r, c] == OCCUPIED:
                color = (120, 58, 66)
            elif grid[r, c] == FREE:
                color = (64, 78, 104) if (c, r) not in frontier else (48, 128, 148)
            else:
                color = (13, 19, 36)
            pygame.draw.rect(surface, color, (x, y, cw_, ch_))


def _draw_scene(snap: dict, surface, title: str, note: str) -> None:
    import pygame
    w, h = surface.get_size()
    surface.fill((11, 16, 32))
    _draw_grid(surface, snap, w, h)

    pad = 40.0
    cw = (w - 2 * pad) / GRID_W
    ch = (h - 2 * pad) / GRID_H

    def to_px(x: float, y: float):
        return pad + x / RES * cw, pad + y / RES * ch

    # 轨迹
    if len(snap["trail"]) > 1:
        pts = [to_px(x, y) for x, y in snap["trail"]]
        pygame.draw.lines(surface, (76, 201, 240), False, pts, 2)

    status = snap["status"]
    # 选中目标
    if snap["target"] is not None:
        tx, ty = snap["target"]["x"], snap["target"]["y"]
        px, py = to_px(tx, ty)
        pygame.draw.circle(surface, (120, 230, 140), (int(px), int(py)), 8, 2)
        pygame.draw.circle(surface, (120, 230, 140), (int(px), int(py)), 3)

    # 无人机
    ux, uy = status["x"], status["y"]
    px, py = to_px(ux, uy)
    yaw = status["yaw"]
    dx, dy = np.cos(yaw) * 12, np.sin(yaw) * 12
    pygame.draw.line(surface, (255, 209, 102),
                     (px - dx, py - dy), (px + dx, py + dy), 4)
    pygame.draw.circle(surface, (255, 255, 255), (int(px), int(py)), 5)

    score = float(snap["scores"].max()) if len(snap["scores"]) else 0.0
    font = _load_font(18)
    small = _load_font(14)
    lines = [
        f"候选前沿 : {status['n_candidates']}",
        f"已探索自由格 : {status['free']}    未知格 : {status['unknown']}",
        f"前沿格数 : {status['frontiers']}    目标评分 : {score:.2f}",
        f"位姿 x/y : {status['x']:.2f} / {status['y']:.2f} m",
    ]
    surface.blit(font.render(title, True, (232, 234, 240)), (24, 14))
    for i, line in enumerate(lines):
        surface.blit(small.render(line, True, (200, 206, 220)), (24, 48 + i * 22))
    if note:
        surface.blit(small.render(note, True, (150, 158, 178)), (24, h - 28))


def render_frame(snap: dict, title: str, note: str,
                 size: tuple[int, int] = (960, 600)) -> "pygame.Surface":
    import pygame
    surface = pygame.Surface(size)
    _draw_scene(snap, surface, title, note)
    return surface


def render_timeline(history: list[dict], panels: int = 6) -> "pygame.Surface":
    import pygame
    cols, rows = 3, 2
    pw, ph = 320, 210
    canvas = pygame.Surface((cols * pw, rows * ph))
    picks = np.linspace(0, len(history) - 1, panels).astype(int)
    for idx, hist_idx in enumerate(picks):
        panel = pygame.Surface((pw, ph))
        _draw_scene(history[hist_idx], panel,
                    f"step {history[hist_idx]['status']['steps']}", "全过程时间轴分帧")
        r, c = divmod(idx, cols)
        canvas.blit(panel, (c * pw, r * ph))
    return canvas


def run_shots() -> dict:
    import pygame
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()

    sim = Simulator()
    sim.run(200)
    snap1 = sim.history[min(1, len(sim.history) - 1)]
    sim.run(200)
    snap2 = sim.history[min(len(sim.history) // 2, len(sim.history) - 1)]
    sim.run(200)
    snap3 = sim.history[-1]

    metrics = sim.finalize()
    os.makedirs(IMG_DIR, exist_ok=True)

    overview = render_frame(snap3, "占据栅格前沿探索全过程总览",
                            "深蓝未知、灰蓝自由、红块障碍、青色前沿、蓝色轨迹")
    pygame.image.save(overview, os.path.join(
        IMG_DIR, "uav_frontier_exploration_occupancy_nn_overview.png"))

    timeline = render_timeline(sim.history)
    pygame.image.save(timeline, os.path.join(
        IMG_DIR, "uav_frontier_exploration_occupancy_nn_timeline.png"))

    keyframes = [
        ("uav_frontier_exploration_occupancy_nn_01_start.png",
         "场景 1：起步建图", "射线探测标出近旁自由格，提取首批前沿候选", snap1),
        ("uav_frontier_exploration_occupancy_nn_02_select.png",
         "场景 2：网络选点", "两层 MLP 按距离、方位、未知邻域与密度打分选中目标前沿", snap2),
        ("uav_frontier_exploration_occupancy_nn_03_explore.png",
         "场景 3：绕行探索", "控制器朝目标前沿推进，避开障碍并扩大已探索区域", snap3),
    ]
    for filename, title, note, snap in keyframes:
        frame = render_frame(snap, title, note)
        pygame.image.save(frame, os.path.join(IMG_DIR, filename))
        print("已保存截图:", os.path.join(IMG_DIR, filename))

    pygame.quit()
    return metrics


def run_demo(duration: float) -> None:  # pragma: no cover - 需图形界面
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((960, 600))
    pygame.display.set_caption("UAV Frontier Exploration Occupancy NN")
    clock = pygame.time.Clock()
    sim = Simulator()
    elapsed = 0.0
    running = True
    while running and elapsed < duration:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        status = sim.step()
        elapsed += 1.0 / 20.0
        snap = sim.history[-1] if sim.history else sim._snapshot(status)
        frame = render_frame(snap, "前沿探索模式",
                             "占据栅格建图、前沿提取与神经网络选目标")
        screen.blit(frame, (0, 0))
        pygame.display.flip()
        clock.tick(20)
    pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description="占据栅格前沿探索神经网络仿真")
    parser.add_argument("--shots", action="store_true", help="离屏运行并保存全过程截图")
    parser.add_argument("--demo", action="store_true", help="打开交互观察窗口")
    parser.add_argument("--duration", type=float, default=30.0, help="交互模式时长 s")
    args = parser.parse_args()

    if args.shots:
        metrics = run_shots()
        print("仿真指标:", metrics)
    elif args.demo:
        run_demo(args.duration)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
