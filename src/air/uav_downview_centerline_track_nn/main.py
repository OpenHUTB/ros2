#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键实验入口（离线可运行，不依赖 ROS）。

用法:
    python main.py --shots      # 离屏运行闭环仿真，保存全过程截图
    python main.py --demo       # 打开交互窗口观察循迹过程
    python main.py --duration 14

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

from uav_downview_centerline_track_nn.controller import TrackController, TrackState  # noqa: E402
from uav_downview_centerline_track_nn.perception import DownviewCamera  # noqa: E402
from uav_downview_centerline_track_nn.policy import TrackPolicyNetwork  # noqa: E402
from uav_downview_centerline_track_nn import track  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(_PKG_DIR, "..", "..", ".."))
IMG_DIR = os.path.join(REPO_ROOT, "docs", "img")

DT = 1.0 / 50.0
RECORD_EVERY = 5
X_MIN, X_MAX = -0.5, 14.5
Y_MIN, Y_MAX = -1.8, 1.8


class Simulator:
    """下视循迹仿真器，记录轨迹、历史状态与关键指标。"""

    def __init__(self) -> None:
        self.camera = DownviewCamera()
        self.policy = TrackPolicyNetwork()
        self.controller = TrackController()
        self.state = TrackState(x=0.0, y=1.2, yaw=0.0)
        self.trail: list[tuple[float, float]] = [(0.0, 1.2)]
        self.history: list[dict] = []
        self.metrics = {"max_abs_lateral": 0.0, "final_lateral": 0.0,
                        "rms_lateral_late": 0.0, "x_reached": 0.0,
                        "max_speed": 0.0}
        self._late_errors: list[float] = []
        self._step = 0

    def step(self) -> None:
        percept = self.camera.observe(self.state)
        speed = float(np.hypot(self.state.vx, self.state.vy))
        feat = DownviewCamera.normalize(percept["offset"], percept["heading"], speed)
        policy_out = self.policy.forward(feat)
        self.state = self.controller.step(self.state, policy_out, DT)

        err = track.lateral_error(self.state.x, self.state.y)
        self.metrics["max_abs_lateral"] = max(self.metrics["max_abs_lateral"], abs(err))
        self.metrics["max_speed"] = max(self.metrics["max_speed"], speed)
        self.metrics["x_reached"] = max(self.metrics["x_reached"], self.state.x)
        if self._step * DT >= 3.0:
            self._late_errors.append(err)

        self.trail.append((float(self.state.x), float(self.state.y)))
        self._step += 1
        if self._step % RECORD_EVERY == 0:
            self.history.append({
                "state": TrackState(self.state.x, self.state.y, self.state.yaw,
                                    self.state.vx, self.state.vy, self.state.altitude),
                "trail": list(self.trail),
                "percept": percept,
                "t": self._step * DT,
            })

    def run(self, seconds: float) -> None:
        for _ in range(int(seconds / DT)):
            self.step()

    def finalize(self) -> dict:
        self.metrics["final_lateral"] = track.lateral_error(self.state.x, self.state.y)
        if self._late_errors:
            self.metrics["rms_lateral_late"] = float(
                np.sqrt(np.mean(np.square(self._late_errors))))
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


def _draw_scene(sim: Simulator, surface, title: str, note: str) -> None:
    import pygame

    w, h = surface.get_size()
    surface.fill((11, 16, 32))

    # 参考标线
    ref_pts = [_to_px(x, track.reference_y(x), w, h)
               for x in np.linspace(track.X_MIN, track.X_MAX, 120)]
    pygame.draw.lines(surface, (90, 98, 116), False, ref_pts, 2)

    # 飞行轨迹
    if len(sim.trail) > 1:
        pts = [_to_px(x, y, w, h) for x, y in sim.trail]
        pygame.draw.lines(surface, (76, 201, 240), False, pts, 2)

    # 无人机
    px, py = _to_px(sim.state.x, sim.state.y, w, h)
    yaw = sim.state.yaw
    fx, fy = np.cos(yaw) * 16, np.sin(yaw) * 16
    pygame.draw.line(surface, (255, 209, 102),
                     (px - fx, py - fy), (px + fx, py + fy), 5)
    pygame.draw.circle(surface, (255, 255, 255), (int(px), int(py)), 6)

    # 下视图像缩略图（右上角）
    percept = sim.history[-1]["percept"] if sim.history else \
        DownviewCamera().observe(sim.state)
    img = percept["image"]
    iw, ih = 120, 90
    ox, oy = w - iw - 18, 18
    pygame.draw.rect(surface, (24, 30, 48), (ox - 4, oy - 4, iw + 8, ih + 8), 1)
    for r in range(img.shape[0]):
        for c in range(img.shape[1]):
            if img[r, c]:
                cw, ch = iw / img.shape[1], ih / img.shape[0]
                pygame.draw.rect(surface, (120, 220, 160),
                                 (ox + c * cw, oy + r * ch, cw, ch))

    offset = track.lateral_error(sim.state.x, sim.state.y)
    speed = float(np.hypot(sim.state.vx, sim.state.vy))
    font = _load_font(18)
    small = _load_font(14)
    lines = [
        f"横向偏移 : {offset:5.2f} m",
        f"航向误差 : {percept['heading']*57.3:5.1f} deg",
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
    sim.run(4.0)
    snap2 = {"state": _state_copy(sim.state), "trail": list(sim.trail)}
    sim.run(8.0)
    snap3 = {"state": _state_copy(sim.state), "trail": list(sim.trail)}

    metrics = sim.finalize()
    os.makedirs(IMG_DIR, exist_ok=True)

    overview = render_frame(sim, "下视循迹全过程总览",
                            "灰色为参考标线，蓝色为飞行轨迹")
    pygame.image.save(overview, os.path.join(
        IMG_DIR, "uav_downview_centerline_track_nn_overview.png"))

    timeline = render_timeline(sim.history)
    pygame.image.save(timeline, os.path.join(
        IMG_DIR, "uav_downview_centerline_track_nn_timeline.png"))

    keyframes = [
        ("uav_downview_centerline_track_nn_01_offline.png",
         "场景 1：偏离起步", "起步时机体偏离标线，下视图像标线偏在一侧", snap1),
        ("uav_downview_centerline_track_nn_02_align.png",
         "场景 2：对齐收敛", "网络输出偏航修正，航向与横向误差逐步收敛", snap2),
        ("uav_downview_centerline_track_nn_03_track.png",
         "场景 3：沿线跟踪", "机体贴合 S 形标线稳定前进", snap3),
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


def _state_copy(state: TrackState) -> TrackState:
    return TrackState(state.x, state.y, state.yaw, state.vx, state.vy, state.altitude)


def run_demo(duration: float) -> None:  # pragma: no cover - 需图形界面
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((960, 600))
    pygame.display.set_caption("UAV Downview Centerline Track NN")
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
        frame = render_frame(sim, "下视循迹模式", "神经网络根据下视图像输出前飞与偏航指令")
        screen.blit(frame, (0, 0))
        pygame.display.flip()
        clock.tick(50)
    pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description="下视摄像头神经网络标线循迹仿真")
    parser.add_argument("--shots", action="store_true", help="离屏运行并保存全过程截图")
    parser.add_argument("--demo", action="store_true", help="打开交互观察窗口")
    parser.add_argument("--duration", type=float, default=14.0, help="交互模式时长 s")
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
