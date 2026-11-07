#!/usr/bin/env python3
"""可交互的无信号环岛车辆运动学仿真。"""

from __future__ import annotations

import argparse
import json
import math
import random
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from carla_nn_roundabout_entry.policy import GapPolicy
from carla_nn_roundabout_entry.sensors import closest_gap, raycast_ranges
from carla_nn_roundabout_entry.slam_nav import OccupancyGrid
from carla_nn_roundabout_entry.end_to_end import render_state_observation, train_and_evaluate_teacher
from carla_nn_roundabout_entry.train_policy import training_data


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
    return run_headless_with_controller(seed, steps, controller="gap-policy")


@lru_cache(maxsize=1)
def _trained_gap_policy() -> GapPolicy:
    features, labels = training_data(seed=7, samples=500)
    policy = GapPolicy(seed=7)
    policy.train(features, labels)
    return policy


@lru_cache(maxsize=1)
def _trained_image_model():
    model, _ = train_and_evaluate_teacher(seed=11, samples=256, image_size=16, hidden=24, epochs=400)
    return model


class ObservationMapTracker:
    """Integrate synthetic range observations into an occupancy grid and run A*."""

    def __init__(self, resolution: float = 0.5) -> None:
        self.resolution = resolution
        self.origin = (-20.0, -20.0)
        self.grid = OccupancyGrid(80, 80)
        self.ray_count = 0
        self.path_search_frames = 0
        self.path_found_frames = 0
        self.last_path_length_cells = 0

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        return (
            int(round((x - self.origin[0]) / self.resolution)),
            int(round((y - self.origin[1]) / self.resolution)),
        )

    def observe(self, world: RoundaboutWorld) -> None:
        ego = world.ego
        circles = [(*car.position(*world.center), 1.1) for car in world.traffic]
        ranges = raycast_ranges((ego.x, ego.y), ego.yaw, circles)
        start = self._cell(ego.x, ego.y)
        ray_count = len(ranges)
        for index, distance in enumerate(ranges):
            angle = ego.yaw - math.pi / 2 + math.pi * index / (ray_count - 1)
            end_x = ego.x + distance * math.cos(angle)
            end_y = ego.y + distance * math.sin(angle)
            hit = distance < 30.0 - 1e-9
            self.grid.integrate_ray(start, self._cell(end_x, end_y), hit=hit)
        self.ray_count += ray_count

        center_distance = min(ranges[ray_count // 2], 8.0)
        goal = self._cell(
            ego.x + center_distance * math.cos(ego.yaw),
            ego.y + center_distance * math.sin(ego.yaw),
        )
        path = self.grid.plan(start, goal)
        self.path_search_frames += 1
        self.last_path_length_cells = len(path)
        if path:
            self.path_found_frames += 1

    def metrics(self) -> dict[str, int | float]:
        cells = [cell for row in self.grid.cells for cell in row]
        return {
            "map_resolution_m": self.resolution,
            "map_rays_integrated": self.ray_count,
            "map_free_cells": cells.count(OccupancyGrid.FREE),
            "map_occupied_cells": cells.count(OccupancyGrid.OCCUPIED),
            "path_search_frames": self.path_search_frames,
            "path_found_frames": self.path_found_frames,
            "path_length_cells": self.last_path_length_cells,
        }


class ImageModelController:
    """Drive the synthetic world from a rendered state observation and MLP output."""

    def __init__(self) -> None:
        self.model = _trained_image_model()
        self.control_frames = 0

    def command(self, world: RoundaboutWorld) -> tuple[float, float, float]:
        ego = world.ego
        traffic = []
        for car in world.traffic:
            x, y = car.position(*world.center)
            traffic.append((x, y, -car.speed * math.sin(car.angle), car.speed * math.cos(car.angle)))
        image = render_state_observation(
            (ego.x, ego.y, ego.yaw, ego.speed), traffic, image_size=16
        )
        steer, throttle, brake = self.model.predict(image[None, ...])[0]
        self.control_frames += 1
        return float(throttle), float(brake), float(steer)

    def metrics(self) -> dict[str, int]:
        return {"image_control_frames": self.control_frames}


class GapAcceptanceController:
    """Use simulated range observations and a trained MLP for each control step."""

    def __init__(self) -> None:
        self.policy = _trained_gap_policy()
        self.wait_frames = 0
        self.enter_frames = 0
        self.committed_enter_frames = 0
        self.emergency_brake_frames = 0
        self.merge_steps_remaining = 0

    def command(self, world: RoundaboutWorld, step_index: int) -> tuple[float, float, float]:
        ego = world.ego
        circles = []
        traffic = []
        for car in world.traffic:
            x, y = car.position(*world.center)
            vx = -car.speed * math.sin(car.angle)
            vy = car.speed * math.cos(car.angle)
            circles.append((x, y, 1.1))
            traffic.append((x, y, vx, vy))

        ranges = raycast_ranges((ego.x, ego.y), ego.yaw, circles)
        gap = closest_gap((ego.x, ego.y, ego.yaw, ego.speed), traffic)
        front_range = min(ranges[10:15])
        clearance = max(0.0, min(gap["gap_m"] - 2.2, front_range))
        features = np.array([
            min(clearance, 20.0) / 20.0,
            min(gap["ttc_s"], 12.0) / 12.0,
            ego.speed / 12.0,
            max(-4.0, min(4.0, gap["lateral_m"])) / 4.0,
        ])
        decision = self.policy.decide(features)

        if decision == "ENTER":
            self.enter_frames += 1
        else:
            self.wait_frames += 1

        if self.merge_steps_remaining == 0 and decision == "ENTER" and clearance >= 6.0 and gap["ttc_s"] >= 3.0:
            self.merge_steps_remaining = 120

        emergency = front_range <= 4.0 or gap["ttc_s"] <= 1.5
        if emergency:
            self.emergency_brake_frames += 1
            throttle, brake = 0.0, 1.0
        elif self.merge_steps_remaining > 0:
            self.committed_enter_frames += 1
            throttle, brake = 0.45, 0.0
            self.merge_steps_remaining -= 1
        elif decision == "ENTER" and clearance >= 6.0 and gap["ttc_s"] >= 3.0:
            throttle, brake = 0.45, 0.0
        else:
            throttle, brake = 0.0, 0.7 if ego.speed > 0.2 else 0.0

        steer = 0.28 if 80 <= step_index < 240 else 0.0
        return throttle, brake, steer

    def metrics(self) -> dict[str, int]:
        return {
            "policy_wait_frames": self.wait_frames,
            "policy_enter_frames": self.enter_frames,
            "committed_enter_frames": self.committed_enter_frames,
            "emergency_brake_frames": self.emergency_brake_frames,
        }


def run_headless_with_controller(
    seed: int,
    steps: int,
    controller: str = "gap-policy",
) -> dict[str, float | int]:
    if steps <= 0:
        raise ValueError("仿真步数必须为正数")
    if controller not in {"gap-policy", "baseline", "image-model"}:
        raise ValueError("控制器必须为 gap-policy、image-model 或 baseline")

    world = RoundaboutWorld(seed=seed)
    policy_controller = GapAcceptanceController() if controller == "gap-policy" else None
    image_controller = ImageModelController() if controller == "image-model" else None
    map_tracker = ObservationMapTracker()
    for index in range(steps):
        map_tracker.observe(world)
        if policy_controller is None:
            if image_controller is None:
                throttle, brake = 0.45, 0.0
                steer = 0.28 if 80 <= index < 240 else 0.0
            else:
                throttle, brake, steer = image_controller.command(world)
        else:
            throttle, brake, steer = policy_controller.command(world, index)
        world.step(throttle=throttle, brake=brake, steer=steer)
    metrics = world.metrics()
    metrics["controller_policy_frames"] = 0 if policy_controller is None else steps
    metrics.update(map_tracker.metrics())
    if policy_controller is not None:
        metrics.update(policy_controller.metrics())
    if image_controller is not None:
        metrics.update(image_controller.metrics())
    return metrics


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
    parser.add_argument("--controller", choices=("gap-policy", "image-model", "baseline"), default="gap-policy")
    args = parser.parse_args()
    if args.gui:
        run_gui(args.seed)
    else:
        result = run_headless_with_controller(args.seed, args.steps, args.controller)
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
