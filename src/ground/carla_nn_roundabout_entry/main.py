#!/usr/bin/env python3
"""可交互的无信号环岛车辆运动学仿真。"""

from __future__ import annotations

import argparse
import json
import math
import random
from dataclasses import dataclass


@dataclass
class Vehicle:
    x: float = -18.0
    y: float = -2.0
    yaw: float = 0.0
    speed: float = 0.0


@dataclass
class TrafficCar:
    angle: float
    speed: float
    radius: float = 8.5

    def position(self, cx: float, cy: float) -> tuple[float, float]:
        return cx + self.radius * math.cos(self.angle), cy + self.radius * math.sin(self.angle)


class RoundaboutWorld:
    """Bicycle-model ego vehicle and deterministic circulating traffic."""

    def __init__(self, seed: int = 7, dt: float = 0.05) -> None:
        self.seed = seed
        self.dt = dt
        self.center = (0.0, 0.0)
        self.ego = Vehicle()
        rng = random.Random(seed)
        self.traffic = [
            TrafficCar(angle, rng.uniform(2.0, 3.0))
            for angle in (-1.1, 0.4, 2.2, 3.7)
        ]
        self.elapsed = 0.0
        self.collisions = 0
        self.min_clearance = float("inf")

    def step(self, throttle: float, brake: float, steer: float) -> None:
        dt = self.dt
        ego = self.ego
        throttle = max(0.0, min(1.0, throttle))
        brake = max(0.0, min(1.0, brake))
        steer = max(-0.55, min(0.55, steer))
        ego.speed = max(0.0, min(12.0, ego.speed + (3.0 * throttle - 7.0 * brake) * dt))
        ego.yaw += ego.speed / 2.7 * math.tan(steer) * dt
        ego.x += ego.speed * math.cos(ego.yaw) * dt
        ego.y += ego.speed * math.sin(ego.yaw) * dt
        for car in self.traffic:
            car.angle += car.speed / car.radius * dt
        self.elapsed += dt
        self._update_clearance()

    def _update_clearance(self) -> None:
        ex, ey = self.ego.x, self.ego.y
        for car in self.traffic:
            x, y = car.position(*self.center)
            clearance = math.hypot(x - ex, y - ey) - 2.2
            self.min_clearance = min(self.min_clearance, clearance)
            if clearance <= 0.0:
                self.collisions += 1

    def metrics(self) -> dict[str, float | int]:
        return {
            "elapsed_s": round(self.elapsed, 3),
            "ego_x_m": round(self.ego.x, 3),
            "ego_y_m": round(self.ego.y, 3),
            "ego_speed_mps": round(self.ego.speed, 3),
            "collision_frames": self.collisions,
            "minimum_clearance_m": round(self.min_clearance, 3),
        }


def run_headless(seed: int, steps: int) -> dict[str, float | int]:
    world = RoundaboutWorld(seed=seed)
    for index in range(steps):
        steer = 0.28 if 80 <= index < 240 else 0.0
        world.step(throttle=0.45, brake=0.0, steer=steer)
    return world.metrics()


def run_gui(seed: int) -> None:
    import tkinter as tk

    world = RoundaboutWorld(seed=seed)
    held: set[str] = set()
    root = tk.Tk()
    root.title("无信号环岛汇入仿真")
    canvas = tk.Canvas(root, width=900, height=700, background="#f4f5f7")
    canvas.pack()
    scale, cx, cy = 23.0, 450.0, 345.0

    def key_down(event: tk.Event) -> None:
        held.add(event.keysym.lower())

    def key_up(event: tk.Event) -> None:
        held.discard(event.keysym.lower())

    root.bind("<KeyPress>", key_down)
    root.bind("<KeyRelease>", key_up)

    def draw_car(x: float, y: float, color: str, radius: float = 0.8) -> None:
        px, py = cx + x * scale, cy - y * scale
        r = radius * scale
        canvas.create_rectangle(px - r, py - r * 0.55, px + r, py + r * 0.55, fill=color, outline="")

    def tick() -> None:
        throttle = float("up" in held or "w" in held)
        brake = float("down" in held or "s" in held)
        steer = float("left" in held or "a" in held) * -0.32
        steer += float("right" in held or "d" in held) * 0.32
        world.step(throttle, brake, steer)
        canvas.delete("all")
        canvas.create_oval(cx - 10 * scale, cy - 10 * scale, cx + 10 * scale, cy + 10 * scale,
                           fill="#d8dce2", outline="#4b5563", width=2)
        canvas.create_oval(cx - 6 * scale, cy - 6 * scale, cx + 6 * scale, cy + 6 * scale,
                           fill="#f4f5f7", outline="#4b5563", width=2)
        canvas.create_rectangle(cx - 24 * scale, cy - 3 * scale, cx - 6 * scale, cy + 3 * scale,
                                fill="#d8dce2", outline="#4b5563", width=2)
        for car in world.traffic:
            draw_car(*car.position(*world.center), "#d97706")
        draw_car(world.ego.x, world.ego.y, "#047857", 1.0)
        metrics = world.metrics()
        canvas.create_text(12, 12, anchor="nw", fill="#111827",
                           text=f"WASD/方向键控制  速度 {metrics['ego_speed_mps']:.1f} m/s  最小间距 {metrics['minimum_clearance_m']:.1f} m")
        root.after(int(world.dt * 1000), tick)

    tick()
    root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="无信号环岛车辆运动学仿真")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--steps", type=int, default=360)
    parser.add_argument("--gui", action="store_true", help="启动键盘交互窗口")
    args = parser.parse_args()
    if args.gui:
        run_gui(args.seed)
    else:
        print(json.dumps(run_headless(args.seed, args.steps), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
