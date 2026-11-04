#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pygame 可视化（**可选依赖**）：保留原有的交互体验。

无图形界面时不要用这个入口（默认 `python3 main.py` 就是离线批跑）。
SDL 没有显示器时可用 ``SDL_VIDEODRIVER=dummy`` 做冒烟测试（验证渲染代码不崩）。
"""
from __future__ import annotations

import math

BG = (30, 30, 30)
GRID = (50, 50, 50)
CAR_COLOR = (60, 160, 255)
TARGET_COLOR = (230, 70, 70)
ARRIVED_COLOR = (80, 220, 120)
OBSTACLE_COLOR = (120, 120, 130)
TRACK_COLOR = (60, 140, 80)
TEXT_COLOR = (230, 230, 230)


def _pygame():
    try:
        import pygame
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            "GUI 模式需要 pygame：pip install pygame\n"
            "无图形界面时请用默认的离线批跑（python3 main.py）"
        ) from exc
    return pygame


def run_gui(cfg, mode="escape", max_frames=None):
    """打开窗口跑交互仿真。``max_frames`` 用于无头冒烟测试（跑够帧数就退出）。"""
    pygame = _pygame()

    from car_2d_target_tracker.car import Car
    from car_2d_target_tracker.controller import Tracker
    from car_2d_target_tracker.target import Target

    pygame.init()
    screen = pygame.display.set_mode((int(cfg.width), int(cfg.height)))
    pygame.display.set_caption("2D Car Target Tracker - SPACE switch / mouse set target")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    car = Car(cfg.width / 2, cfg.height / 2, 0.0, radius=cfg.car_radius)
    target = Target(700, 150, speed=cfg.target_speed, width=cfg.width,
                    height=cfg.height, margin=cfg.target_margin, seed=cfg.seed)
    ctrl = Tracker(cfg, mode=mode)
    auto = True
    arrived = False
    frame = 0

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        frame += 1
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                target.set_pos(*event.pos)
                ctrl.reset()
                arrived = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    auto = not auto
                    ctrl.reset()
                    arrived = False
                elif event.key == pygame.K_r:
                    car.reset(cfg.width / 2, cfg.height / 2, 0.0)
                    ctrl.reset()
                    arrived = False

        keys = pygame.key.get_pressed()
        obstacles = cfg.obstacles
        if auto:
            v, omega, arrived = ctrl.compute(car, target)
            car.step(v, omega, dt, obstacles)
            target.step(dt)
        else:
            v = omega = 0.0
            if keys[pygame.K_UP] or keys[pygame.K_w]:
                v = cfg.max_speed
            elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
                v = -cfg.max_speed * 0.6
            if keys[pygame.K_LEFT] or keys[pygame.K_a]:
                omega = cfg.max_steer
            elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
                omega = -cfg.max_steer
            car.step(v, omega, dt, obstacles)
            target.step(dt)

        _draw(pygame, screen, font, cfg, car, target, auto, arrived, obstacles)
        pygame.display.flip()

        if max_frames is not None and frame >= max_frames:
            running = False

    pygame.quit()
    return 0


def _draw(pygame, screen, font, cfg, car, target, auto, arrived, obstacles):
    screen.fill(BG)
    for x in range(0, int(cfg.width), 45):
        pygame.draw.line(screen, GRID, (x, 0), (x, int(cfg.height)))
    for y in range(0, int(cfg.height), 45):
        pygame.draw.line(screen, GRID, (0, y), (int(cfg.width), y))

    for (ox, oy, r) in obstacles:
        pygame.draw.circle(screen, OBSTACLE_COLOR, (int(ox), int(oy)), int(r))
        pygame.draw.circle(screen, (80, 80, 90), (int(ox), int(oy)), int(r), 2)

    if len(car.trail) > 1:
        pygame.draw.lines(screen, TRACK_COLOR, False,
                          [(int(px), int(py)) for px, py in car.trail], 2)

    tcol = ARRIVED_COLOR if arrived else TARGET_COLOR
    pygame.draw.circle(screen, tcol, (int(target.x), int(target.y)), 12, 0)
    pygame.draw.circle(screen, tcol, (int(target.x), int(target.y)), 18, 2)

    size = 16
    pts = [(car.x + size * math.cos(car.theta + a),
            car.y + size * math.sin(car.theta + a)) for a in (0.0, 2.5, -2.5)]
    pygame.draw.polygon(screen, CAR_COLOR, [(int(px), int(py)) for px, py in pts])
    pygame.draw.circle(screen, (200, 200, 200), (int(car.x), int(car.y)), 3)
    pygame.draw.line(screen, (100, 100, 100), (car.x, car.y), (target.x, target.y), 1)

    lines = ["mode: " + ("AUTO (tracking)" if auto else "MANUAL (WASD/arrows)"),
             "speed: %6.1f px/s" % car.v,
             "dist:  %6.1f px" % math.hypot(target.x - car.x, target.y - car.y),
             "",
             "[SPACE] switch mode   [mouse] set target   [R] reset   [ESC] quit"]
    for i, line in enumerate(lines):
        color = ARRIVED_COLOR if (i == 0 and auto) else TEXT_COLOR
        screen.blit(font.render(line, True, color), (12, 12 + i * 22))
