#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键实验入口（离线可运行，不依赖 ROS）。

用法:
    python main.py --shots      # 离屏运行三段仿真场景，保存全过程截图
    python main.py --demo       # 打开交互窗口，方向键/ WASD / QE 遥控
    python main.py --duration 8 # 交互模式下仿真时长（秒）

截图输出到仓库 docs/img/ 目录，文件名前缀与功能包一致：
    - 全程轨迹总览图；
    - 全过程多时间轴分帧拼图；
    - 三个阶段结束时刻的关键帧。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

# 允许直接以脚本方式运行：把包目录加入 sys.path
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from uav_rigidbody_keyboard_teleop.dynamics import Quadrotor, State  # noqa: E402
from uav_rigidbody_keyboard_teleop.controller import TeleopController  # noqa: E402
from uav_rigidbody_keyboard_teleop.teleop import keys_to_cmd  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(_PKG_DIR, "..", "..", ".."))
IMG_DIR = os.path.join(REPO_ROOT, "docs", "img")

DT = 1.0 / 250.0
WORLD_X_MIN, WORLD_X_MAX = -1.0, 8.0
WORLD_Z_MAX = 2.5
RECORD_EVERY = 10  # 每 10 步记录一帧历史


class Simulator:
    """四旋翼仿真器，记录轨迹、历史状态与关键指标。"""

    def __init__(self) -> None:
        self.robot = Quadrotor()
        self.controller = TeleopController()
        self.state = State()
        self.trail: list[tuple[float, float]] = [(0.0, 0.0)]
        self.history: list[dict] = []
        self.metrics = {"max_altitude": 0.0, "forward_distance": 0.0,
                        "max_speed": 0.0}
        self._step = 0

    def step(self, cmd: dict) -> None:
        wrench = self.controller.update(self.state, cmd)
        self.state = self.robot.step(self.state, wrench, DT)
        self.trail.append((float(self.state.pos[0]), float(self.state.pos[2])))
        self.metrics["max_altitude"] = max(self.metrics["max_altitude"],
                                           float(self.state.pos[2]))
        self.metrics["forward_distance"] = max(
            self.metrics["forward_distance"], float(self.state.pos[0]))
        self.metrics["max_speed"] = max(
            self.metrics["max_speed"], float(np.linalg.norm(self.state.vel)))
        self._step += 1
        if self._step % RECORD_EVERY == 0:
            self.history.append({
                "state": self.state.copy(),
                "trail": list(self.trail),
                "t": self._step * DT,
            })

    def run_phase(self, cmd: dict, seconds: float) -> None:
        for _ in range(int(seconds / DT)):
            self.step(cmd)


def _load_font(size: int):
    import pygame
    for path in (r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\msyh.ttc"):
        if os.path.exists(path):
            return pygame.font.Font(path, size)
    return pygame.font.Font(None, size)


def _draw_scene(sim: Simulator, surface, title: str, note: str) -> None:
    """在给定 surface 上绘制侧视图（x-z）+ HUD。"""
    import pygame

    W, H = surface.get_size()
    surface.fill((11, 16, 32))
    sx = 100.0
    sz = 80.0
    ground_y = H - 56

    def to_px(x: float, z: float) -> tuple[float, float]:
        px = 120 + (x - WORLD_X_MIN) * sx
        py = ground_y - z * sz
        return px, py

    for gx in range(int(WORLD_X_MIN), int(WORLD_X_MAX) + 1):
        x0, _ = to_px(gx, 0)
        pygame.draw.line(surface, (28, 34, 52), (x0, 36), (x0, ground_y), 1)
    for gz in range(0, int(WORLD_Z_MAX) + 1):
        _, y0 = to_px(0, gz)
        pygame.draw.line(surface, (28, 34, 52), (120, y0), (W - 40, y0), 1)

    pygame.draw.line(surface, (90, 98, 116), (120, ground_y), (W - 40, ground_y), 3)

    if len(sim.trail) > 1:
        pts = [to_px(x, z) for x, z in sim.trail]
        pygame.draw.lines(surface, (76, 201, 240), False, pts, 2)

    x, _, z = sim.state.pos
    px, py = to_px(x, z)
    pitch = sim.state.rpy[1]
    body_len, arm = 54, 26
    bx = body_len * math.cos(pitch)
    by = body_len * math.sin(pitch)
    pygame.draw.line(surface, (255, 209, 102),
                     (px - bx, py - by), (px + bx, py + by), 6)
    for sign in (-1, 1):
        rx = px + sign * arm * math.cos(pitch)
        ry = py + sign * arm * math.sin(pitch)
        pygame.draw.ellipse(surface, (255, 209, 102),
                            (rx - 18, ry - 6, 36, 12), 3)
    pygame.draw.circle(surface, (255, 255, 255), (int(px), int(py)), 7)

    font = _load_font(18)
    small = _load_font(14)
    h, vx, vz = sim.state.pos[2], sim.state.vel[0], sim.state.vel[2]
    roll, pth, yaw = sim.state.rpy
    thrust = float(np.sum(sim.robot.p.k_f * np.clip(
        sim.state.omega_cmd, sim.robot.p.min_omega,
        sim.robot.p.max_omega) ** 2))
    thrust_margin = thrust / (4.0 * sim.robot.hover_thrust())

    lines = [
        f"高度 z : {h:5.2f} m",
        f"水平速度 vx : {vx:5.2f} m/s",
        f"垂直速度 vz : {vz:5.2f} m/s",
        f"滚转/俯仰/偏航 : {roll*57.3:6.1f} / {pth*57.3:6.1f} / {yaw*57.3:6.1f} deg",
        f"推力余量 : {thrust_margin*100:5.1f} %",
    ]
    surface.blit(font.render(title, True, (232, 234, 240)), (30, 18))
    for i, line in enumerate(lines):
        surface.blit(small.render(line, True, (200, 206, 220)), (30, 56 + i * 22))


def render_frame(sim: Simulator, title: str, note: str,
                 size: tuple[int, int] = (960, 600)) -> "pygame.Surface":
    import pygame
    surface = pygame.Surface(size)
    _draw_scene(sim, surface, title, note)
    return surface


def render_timeline(history: list[dict], panels: int = 6) -> "pygame.Surface":
    """把全过程历史均匀抽帧，拼成 2×3 时间轴分帧拼图。"""
    import pygame

    cols, rows = 3, 2
    pw, ph = 320, 210
    canvas = pygame.Surface((cols * pw, rows * ph))
    picks = np.linspace(0, len(history) - 1, panels).astype(int)
    for idx, hist_idx in enumerate(picks):
        snap = history[hist_idx]
        panel = pygame.Surface((pw, ph))
        # 用快照状态/轨迹临时构造仿真器视图
        view = Simulator()
        view.state = snap["state"]
        view.trail = snap["trail"]
        view.robot = view.robot  # 保留参数引用
        _draw_scene(view, panel, f"t = {snap['t']:.2f} s",
                    "全过程时间轴分帧")
        r, c = divmod(idx, cols)
        canvas.blit(panel, (c * pw, r * ph))
    return canvas


def run_shots() -> dict:
    """运行三段场景，保存全过程截图，返回指标摘要。"""
    import pygame
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()

    sim = Simulator()

    # 场景 1：起飞爬升到悬停高度（油门 +0.8）
    sim.run_phase({"throttle": 0.8, "pitch": 0.0, "roll": 0.0, "yaw": 0.0}, 1.6)
    snap1 = {"state": sim.state.copy(), "trail": list(sim.trail)}

    # 场景 2：低头前飞（俯仰 +0.6）
    sim.run_phase({"throttle": 0.0, "pitch": 0.6, "roll": 0.0, "yaw": 0.0}, 2.0)
    snap2 = {"state": sim.state.copy(), "trail": list(sim.trail)}

    # 场景 3：松杆保持（中立指令）
    sim.run_phase({"throttle": 0.0, "pitch": 0.0, "roll": 0.0, "yaw": 0.0}, 2.0)
    snap3 = {"state": sim.state.copy(), "trail": list(sim.trail)}

    os.makedirs(IMG_DIR, exist_ok=True)

    # 1) 全程轨迹总览
    overview = render_frame(sim, "全程仿真过程总览",
                           "起飞爬升 → 低头前飞 → 松杆保持，蓝色为完整飞行轨迹")
    pygame.image.save(overview, os.path.join(
        IMG_DIR, "uav_rigidbody_keyboard_teleop_overview.png"))

    # 2) 全过程时间轴分帧拼图
    timeline = render_timeline(sim.history)
    pygame.image.save(timeline, os.path.join(
        IMG_DIR, "uav_rigidbody_keyboard_teleop_timeline.png"))

    # 3) 三个阶段关键帧
    keyframes = [
        ("uav_rigidbody_keyboard_teleop_01_takeoff.png",
         "场景 1：起飞爬升", "按键：↑ 增加油门，四旋翼离地并爬升到悬停高度", snap1),
        ("uav_rigidbody_keyboard_teleop_02_forward.png",
         "场景 2：低头前飞", "按键：W 低头，机身俯仰产生水平前飞分量", snap2),
        ("uav_rigidbody_keyboard_teleop_03_hold.png",
         "场景 3：松杆保持", "松开所有按键，姿态在阻尼作用下回正并保持悬停", snap3),
    ]
    for filename, title, note, snap in keyframes:
        view = Simulator()
        view.state = snap["state"]
        view.trail = snap["trail"]
        frame = render_frame(view, title, note)
        pygame.image.save(frame, os.path.join(IMG_DIR, filename))
        print("已保存截图:", os.path.join(IMG_DIR, filename))

    pygame.quit()
    return sim.metrics


def run_demo(duration: float) -> None:  # pragma: no cover - 需图形界面
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((960, 600))
    pygame.display.set_caption("UAV Rigidbody Keyboard Teleop")
    clock = pygame.time.Clock()
    sim = Simulator()

    elapsed = 0.0
    running = True
    while running and elapsed < duration:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        keys = {k.name for k in pygame.key.get_pressed() if k}
        cmd = keys_to_cmd(keys)
        sim.step(cmd)
        elapsed += DT
        frame = render_frame(sim, "交互遥控模式",
                             "W/S 俯仰 · A/D 滚转 · ↑/↓ 油门 · Q/E 偏航")
        screen.blit(frame, (0, 0))
        pygame.display.flip()
        clock.tick(250)

    pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description="四旋翼刚体动力学键盘遥控仿真")
    parser.add_argument("--shots", action="store_true", help="离屏运行并保存全过程截图")
    parser.add_argument("--demo", action="store_true", help="打开交互遥控窗口")
    parser.add_argument("--duration", type=float, default=10.0, help="交互模式时长 s")
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
