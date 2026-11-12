#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键实验入口（离线可运行，不依赖 ROS）。

用法:
    python main.py --shots      # 离屏运行闭环仿真，保存全过程截图
    python main.py --demo       # 打开交互窗口观察通行决策过程
    python main.py --duration 19

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

from uav_depth_terrain_passability_nn.controller import TerrainController, TerrainState  # noqa: E402
from uav_depth_terrain_passability_nn.depth_sensor import DepthTerrainSensor  # noqa: E402
from uav_depth_terrain_passability_nn.policy import PassabilityNetwork  # noqa: E402
from uav_depth_terrain_passability_nn import terrain  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(_PKG_DIR, "..", "..", ".."))
IMG_DIR = os.path.join(REPO_ROOT, "docs", "img")

DT = 1.0 / 50.0
RECORD_EVERY = 5
X_MIN, X_MAX = 0.0, 10.0
Y_MIN, Y_MAX = -3.0, 3.0

SECTOR_NAME = ("left", "center", "right")


class Simulator:
    """地形通行度仿真器，记录轨迹、历史状态与关键指标。"""

    def __init__(self) -> None:
        self.sensor = DepthTerrainSensor()
        self.policy = PassabilityNetwork()
        self.controller = TerrainController()
        self.state = TerrainState(x=0.0, y=0.0, yaw=0.0)
        self.trail: list[tuple[float, float]] = [(0.0, 0.0)]
        self.history: list[dict] = []
        self.metrics = {"max_abs_lateral": 0.0, "final_lateral": 0.0,
                        "x_reached": 0.0, "max_speed": 0.0,
                        "block_touch": 0}
        self._step = 0

    def step(self) -> None:
        percept = self.sensor.observe(self.state)
        feat = DepthTerrainSensor.normalize(percept["features"])
        decision = self.policy.forward(feat)
        self.state = self.controller.step(self.state, decision, DT)

        speed = float(np.hypot(self.state.vx, self.state.vy))
        self.metrics["max_abs_lateral"] = max(
            self.metrics["max_abs_lateral"], abs(self.state.y))
        self.metrics["max_speed"] = max(self.metrics["max_speed"], speed)
        self.metrics["x_reached"] = max(self.metrics["x_reached"], self.state.x)
        if terrain.is_blocked(self.state.x, self.state.y):
            self.metrics["block_touch"] += 1

        self.trail.append((float(self.state.x), float(self.state.y)))
        self._step += 1
        if self._step % RECORD_EVERY == 0:
            self.history.append({
                "state": TerrainState(self.state.x, self.state.y, self.state.yaw,
                                      self.state.vx, self.state.vy, self.state.altitude),
                "trail": list(self.trail),
                "decision": decision,
                "t": self._step * DT,
            })

    def run(self, seconds: float) -> None:
        for _ in range(int(seconds / DT)):
            self.step()

    def finalize(self) -> dict:
        self.metrics["final_lateral"] = self.state.y
        return self.metrics


def _load_font(size: int):
    import pygame
    for path in (r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\msyh.ttc"):
        if os.path.exists(path):
            return pygame.font.Font(path, size)
    return pygame.font.Font(None, size)


def _to_px(x: float, y: float, w: int, h: int) -> tuple[float, float]:
    pad_x, pad_y = 70.0, 60.0
    sx = (w - 2 * pad_x) / (X_MAX - X_MIN)
    sy = (h - 2 * pad_y) / (Y_MAX - Y_MIN)
    px = pad_x + (x - X_MIN) * sx
    py = pad_y + (Y_MAX - y) * sy
    return px, py


def _draw_blocks(surface, w: int, h: int) -> None:
    import pygame
    rects = [
        (terrain.BLOCK_X0, terrain.BLOCK_Y0, terrain.BLOCK_X1, terrain.BLOCK_Y1),
        (terrain.BLOCK_X0, terrain.RIGHT_Y0, terrain.BLOCK_X1, terrain.RIGHT_Y1),
    ]
    for x0, y0, x1, y1 in rects:
        p0 = _to_px(x0, y1, w, h)
        p1 = _to_px(x1, y0, w, h)
        pygame.draw.rect(surface, (86, 58, 66),
                         (p0[0], p0[1], p1[0] - p0[0], p1[1] - p0[1]))
        pygame.draw.rect(surface, (140, 92, 102),
                         (p0[0], p0[1], p1[0] - p0[0], p1[1] - p0[1]), 1)


def _draw_scene(sim: Simulator, surface, title: str, note: str) -> None:
    import pygame

    w, h = surface.get_size()
    surface.fill((11, 16, 32))
    _draw_blocks(surface, w, h)

    # 飞行轨迹
    if len(sim.trail) > 1:
        pts = [_to_px(x, y, w, h) for x, y in sim.trail]
        pygame.draw.lines(surface, (76, 201, 240), False, pts, 2)

    # 三扇区前向射线
    fx, fy = sim.state.x, sim.state.y
    yaw = sim.state.yaw
    for sector, lat in enumerate((1.2, 0.0, -1.2)):
        wx = fx + np.cos(yaw) * 2.5 - np.sin(yaw) * lat
        wy = fy + np.sin(yaw) * 2.5 + np.cos(yaw) * lat
        a = _to_px(fx, fy, w, h)
        b = _to_px(wx, wy, w, h)
        pygame.draw.line(surface, (60, 72, 96), a, b, 1)

    # 无人机
    px, py = _to_px(sim.state.x, sim.state.y, w, h)
    yaw = sim.state.yaw
    dx, dy = np.cos(yaw) * 16, np.sin(yaw) * 16
    pygame.draw.line(surface, (255, 209, 102),
                     (px - dx, py - dy), (px + dx, py + dy), 5)
    pygame.draw.circle(surface, (255, 255, 255), (int(px), int(py)), 6)

    decision = sim.history[-1]["decision"] if sim.history else None
    if decision is None:
        percept = DepthTerrainSensor().observe(sim.state)
        decision = sim.policy.forward(DepthTerrainSensor.normalize(percept["features"]))

    p = decision["passability"]
    speed = float(np.hypot(sim.state.vx, sim.state.vy))
    font = _load_font(18)
    small = _load_font(14)
    lines = [
        f"选中扇区 : {SECTOR_NAME[decision['sector']]}",
        f"通行度   : L {p[0]:.2f} / C {p[1]:.2f} / R {p[2]:.2f}",
        f"前飞速度 : {speed:5.2f} m/s",
        f"轨迹 x   : {sim.state.x:5.2f} m",
    ]
    surface.blit(font.render(title, True, (232, 234, 240)), (24, 16))
    for i, line in enumerate(lines):
        surface.blit(small.render(line, True, (200, 206, 220)), (24, 52 + i * 22))
    if note:
        surface.blit(small.render(note, True, (150, 158, 178)), (24, h - 30))


def render_frame(sim: Simulator, title: str, note: str,
                 size: tuple[int, int] = (960, 600)) -> "pygame.Surface":
    import pygame
    surface = pygame.Surface(size)
    _draw_scene(sim, surface, title, note)
    return surface


def render_timeline(history: list[dict], panels: int = 6) -> "pygame.Surface":
    import pygame

    cols, rows = 3, 2
    pw, ph = 320, 210
    canvas = pygame.Surface((cols * pw, rows * ph))
    picks = np.linspace(0, len(history) - 1, panels).astype(int)
    for idx, hist_idx in enumerate(picks):
        snap = history[hist_idx]
        view = Simulator()
        view.state = snap["state"]
        view.trail = snap["trail"]
        panel = pygame.Surface((pw, ph))
        _draw_scene(view, panel, f"t = {snap['t']:.2f} s", "全过程时间轴分帧")
        r, c = divmod(idx, cols)
        canvas.blit(panel, (c * pw, r * ph))
    return canvas


def run_shots() -> dict:
    import pygame
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()

    sim = Simulator()
    sim.run(2.0)
    snap1 = {"state": _state_copy(sim.state), "trail": list(sim.trail)}
    sim.run(5.0)
    snap2 = {"state": _state_copy(sim.state), "trail": list(sim.trail)}
    sim.run(12.0)
    snap3 = {"state": _state_copy(sim.state), "trail": list(sim.trail)}

    metrics = sim.finalize()
    os.makedirs(IMG_DIR, exist_ok=True)

    overview = render_frame(sim, "地形可通行性评估全过程总览",
                            "两侧为抬高台阶障碍，蓝色为飞行轨迹")
    pygame.image.save(overview, os.path.join(
        IMG_DIR, "uav_depth_terrain_passability_nn_overview.png"))

    timeline = render_timeline(sim.history)
    pygame.image.save(timeline, os.path.join(
        IMG_DIR, "uav_depth_terrain_passability_nn_timeline.png"))

    keyframes = [
        ("uav_depth_terrain_passability_nn_01_approach.png",
         "场景 1：接近障碍带", "深度扇区开始读到两侧台阶，中央扇区通行度占优", snap1),
        ("uav_depth_terrain_passability_nn_02_corridor.png",
         "场景 2：锁定中央通道", "网络持续选中中央可通行扇区，机体沿通道直行", snap2),
        ("uav_depth_terrain_passability_nn_03_cross.png",
         "场景 3：穿越障碍带", "机体保持横向居中，从两侧台阶之间穿过", snap3),
    ]
    for filename, title, note, snap in keyframes:
        view = Simulator()
        view.state = snap["state"]
        view.trail = snap["trail"]
        frame = render_frame(view, title, note)
        pygame.image.save(frame, os.path.join(IMG_DIR, filename))
        print("已保存截图:", os.path.join(IMG_DIR, filename))

    pygame.quit()
    return metrics


def _state_copy(state: TerrainState) -> TerrainState:
    return TerrainState(state.x, state.y, state.yaw, state.vx, state.vy, state.altitude)


def run_demo(duration: float) -> None:  # pragma: no cover - 需图形界面
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((960, 600))
    pygame.display.set_caption("UAV Depth Terrain Passability NN")
    clock = pygame.time.Clock()
    sim = Simulator()
    elapsed = 0.0
    running = True
    while running and elapsed < duration:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        sim.step()
        elapsed += DT
        frame = render_frame(sim, "地形通行决策模式",
                             "神经网络根据三扇区地形特征评估通行度并选择通道")
        screen.blit(frame, (0, 0))
        pygame.display.flip()
        clock.tick(50)
    pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description="深度相机地形可通行性神经网络仿真")
    parser.add_argument("--shots", action="store_true", help="离屏运行并保存全过程截图")
    parser.add_argument("--demo", action="store_true", help="打开交互观察窗口")
    parser.add_argument("--duration", type=float, default=19.0, help="交互模式时长 s")
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
