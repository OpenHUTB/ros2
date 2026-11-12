#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键实验入口（离线可运行，不依赖 ROS）。

用法:
    python main.py --shots      # 离屏运行闭环仿真，保存全过程截图
    python main.py --demo       # 打开交互窗口观察端到端跟随过程

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

from uav_e2e_multiframe_image_policy.runner import E2ERunner  # noqa: E402
from uav_e2e_multiframe_image_policy import scene  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(_PKG_DIR, "..", "..", ".."))
IMG_DIR = os.path.join(REPO_ROOT, "docs", "img")

RECORD_EVERY = 6
X_MIN, X_MAX = scene.X_MIN, scene.X_MAX
Y_MIN, Y_MAX = -1.3, 1.3


class Simulator:
    """端到端策略仿真器，封装闭环并记录轨迹快照与指标。"""

    def __init__(self) -> None:
        self.runner = E2ERunner()
        self.history: list[dict] = []

    def step(self) -> dict:
        status = self.runner.step()
        if status["steps"] % RECORD_EVERY == 0:
            self.history.append(self._snapshot(status))
        return status

    def run(self, max_steps: int) -> None:
        for _ in range(max_steps):
            if self.runner.done:
                break
            self.step()

    def _snapshot(self, status: dict) -> dict:
        frame = self.runner.history.frames[-1].copy()
        return {
            "trail": list(self.runner.trajectory),
            "frame": frame,
            "status": dict(status),
        }

    def finalize(self) -> dict:
        return dict(self.runner.metrics)


def _load_font(size: int):
    import pygame
    for path in (r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\msyh.ttc"):
        if os.path.exists(path):
            return pygame.font.Font(path, size)
    return pygame.font.Font(None, size)


def _world_to_px(x: float, y: float, w: int, h: int, pad: float):
    cw = (w - 2 * pad) / (X_MAX - X_MIN)
    ch = (h - 2 * pad) / (Y_MAX - Y_MIN)
    return pad + (x - X_MIN) * cw, pad + (Y_MAX - y) * ch


def _draw_centerline(surface, w: int, h: int, pad: float) -> None:
    import pygame
    pts = []
    for x in np.linspace(X_MIN, X_MAX, 120):
        px, py = _world_to_px(x, scene.centerline_y(x), w, h, pad)
        pts.append((px, py))
    pygame.draw.lines(surface, (90, 110, 150), False, pts, 2)


def _draw_downview_inset(surface, frame: np.ndarray, origin: tuple[int, int],
                         pw: int, ph: int) -> None:
    import pygame
    fh, fw = frame.shape
    for r in range(fh):
        for c in range(fw):
            v = int(round(frame[r, c] * 255))
            rect = (origin[0] + c * pw // fw, origin[1] + r * ph // fh,
                    pw // fw + 1, ph // fh + 1)
            pygame.draw.rect(surface, (v, v, v), rect)


def _draw_scene(snap: dict, surface, title: str, note: str) -> None:
    import pygame
    w, h = surface.get_size()
    surface.fill((11, 16, 32))
    pad = 40.0
    _draw_centerline(surface, w, h, pad)

    trail = snap["trail"]
    if len(trail) > 1:
        pts = [_world_to_px(x, y, w, h, pad) for x, y in trail]
        pygame.draw.lines(surface, (76, 201, 240), False, pts, 2)

    status = snap["status"]
    ux, uy = status["x"], status["y"]
    px, py = _world_to_px(ux, uy, w, h, pad)
    yaw = status["yaw"]
    dx, dy = np.cos(yaw) * 12, np.sin(yaw) * 12
    pygame.draw.line(surface, (255, 209, 102),
                     (px - dx, py - dy), (px + dx, py + dy), 4)
    pygame.draw.circle(surface, (255, 255, 255), (int(px), int(py)), 5)

    inset_w, inset_h = 120, 90
    _draw_downview_inset(surface, snap["frame"],
                         (w - inset_w - 20, h - inset_h - 20), inset_w, inset_h)

    font = _load_font(18)
    small = _load_font(14)
    lines = [
        f"位姿 x/y : {status['x']:.2f} / {status['y']:.2f} m",
        f"横向偏差 : {status['offset']:+.2f} m",
        f"前飞/偏航 : {status['forward']:.2f} / {status['yaw_cmd']:+.2f}",
        f"步数 : {status['steps']}",
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
                    f"step {history[hist_idx]['status']['steps']}",
                    "全过程时间轴分帧")
        r, c = divmod(idx, cols)
        canvas.blit(panel, (c * pw, r * ph))
    return canvas


def run_shots() -> dict:
    import pygame
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()

    sim = Simulator()
    sim.run(420)
    snap1 = sim.history[min(1, len(sim.history) - 1)]
    snap2 = sim.history[min(len(sim.history) // 2, len(sim.history) - 1)]
    snap3 = sim.history[-1]

    metrics = sim.finalize()
    os.makedirs(IMG_DIR, exist_ok=True)

    overview = render_frame(snap3, "多帧图像端到端策略全过程总览",
                            "灰蓝参考中心线、蓝色轨迹、黄色航向、右下角为当前下视灰度图")
    pygame.image.save(overview, os.path.join(
        IMG_DIR, "uav_e2e_multiframe_image_policy_overview.png"))

    timeline = render_timeline(sim.history)
    pygame.image.save(timeline, os.path.join(
        IMG_DIR, "uav_e2e_multiframe_image_policy_timeline.png"))

    keyframes = [
        ("uav_e2e_multiframe_image_policy_01_start.png",
         "场景 1：起步跟线", "三帧下视图像堆叠，策略网络从中心线质心偏差给出初始偏航", snap1),
        ("uav_e2e_multiframe_image_policy_02_track.png",
         "场景 2：弯道跟随", "端到端网络依据中心可见度与横向质心持续修正前飞与偏航", snap2),
        ("uav_e2e_multiframe_image_policy_03_finish.png",
         "场景 3：抵达远端", "沿参考中心线完成全程跟随，右下角保留当前帧下视图", snap3),
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
    pygame.display.set_caption("UAV E2E Multiframe Image Policy")
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
        frame = render_frame(snap, "端到端跟随模式",
                             "多帧下视图像 -> 卷积/MLP 策略 -> 前飞与偏航指令")
        screen.blit(frame, (0, 0))
        pygame.display.flip()
        clock.tick(20)
    pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description="多帧图像端到端策略仿真")
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
