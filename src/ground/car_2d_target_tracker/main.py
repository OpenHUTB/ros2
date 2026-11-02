"""
2D 小车目标追踪仿真（Pygame）
- 空格切换：自动追目标 / 手动键盘控制
- 目标自己会随机移动
- 小车带轨迹线
- 场景里有静态障碍物，自动模式带势场避障（前瞻切向绕行）
- 鼠标点一下：在自动模式下设置新目标位置
- R 重置，ESC 退出
- 加 --demo 参数：自动跑几个场景并保存截图后退出

v2 修复：旧版撞到障碍物后只会原地全锁转向（copysign(MAX_STEER, omega)），
目标在障碍物正后方时形成极限环死锁。核心逻辑已抽出为 tracker_core.py
（人工势场 + 逐障碍切向绕行 + 卡死退避 + 临时绕行点），并用 12 项
单元测试回归覆盖（python -m pytest test_tracker_core.py -v）。
"""
import math
import sys

import pygame

import tracker_core as tc

# ---------- 窗口与颜色 ----------
WIDTH, HEIGHT = 900, 600
BG = (30, 30, 30)
GRID = (50, 50, 50)
CAR_COLOR = (60, 160, 255)
TARGET_COLOR = (230, 70, 70)
ARRIVED_COLOR = (80, 220, 120)
OBSTACLE_COLOR = (120, 120, 130)
TRACK_COLOR = (60, 140, 80)
TEXT_COLOR = (230, 230, 230)

# 静态障碍物（圆心x, 圆心y, 半径）
OBSTACLES = [
    (300, 200, 35),
    (600, 380, 45),
    (450, 450, 30),
]


def draw_grid(screen):
    for x in range(0, WIDTH, 45):
        pygame.draw.line(screen, GRID, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, 45):
        pygame.draw.line(screen, GRID, (0, y), (WIDTH, y))


def draw_obstacles(screen):
    for (ox, oy, r) in OBSTACLES:
        pygame.draw.circle(screen, OBSTACLE_COLOR, (ox, oy), r)
        pygame.draw.circle(screen, (80, 80, 90), (ox, oy), r, 2)


def draw_trail(screen, car):
    if len(car.trail) > 1:
        pygame.draw.lines(screen, TRACK_COLOR, False, list(car.trail), 2)


def draw_hud(screen, car, target, auto_mode, arrived, font):
    mode_str = "AUTO (avoidance on)" if auto_mode else "MANUAL (WASD/arrows)"
    mode_color = ARRIVED_COLOR if auto_mode else (230, 200, 100)
    lines = [
        f"mode: {mode_str}",
        f"speed: {car.v:6.1f} px/s",
        f"angle: {math.degrees(tc.normalize_angle(car.theta)):7.1f} deg",
        f"dist:  {math.hypot(target.x-car.x, target.y-car.y):6.1f} px",
        "",
        "[SPACE] switch mode   [mouse] set target   [R] reset   [ESC] quit",
    ]
    for i, line in enumerate(lines):
        c = mode_color if i == 0 else TEXT_COLOR
        surf = font.render(line, True, c)
        screen.blit(surf, (12, 12 + i * 22))


def make_car(x, y, theta=0.0):
    return tc.Car(x, y, theta, obstacles=OBSTACLES, bounds=(WIDTH, HEIGHT))


def run(demo=False):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("2D Car Target Tracker - SPACE to switch mode")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    car = make_car(WIDTH / 2, HEIGHT / 2, 0.0)
    target = tc.MovingTarget(700, 150, bounds=(WIDTH, HEIGHT))
    auto_mode = True
    arrived = False

    frames = {}  # demo 模式：frame -> 保存文件名
    if demo:
        import random
        random.seed(42)
        frames[150] = "shot_demo_1_auto_chasing.png"
        # frame 300 重置到"目标在障碍物正后方"场景（旧版死锁、本版绕行）
        frames[430] = "shot_demo_2_avoiding.png"
        frames[560] = "shot_demo_3_manual.png"

    running = True
    frame = 0
    while running:
        dt = clock.tick(60) / 1000.0
        frame += 1
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and not demo:
                target.set_pos(*event.pos)
                arrived = False
            elif event.type == pygame.KEYDOWN and not demo:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    auto_mode = not auto_mode
                    arrived = False
                elif event.key == pygame.K_r:
                    car = make_car(WIDTH / 2, HEIGHT / 2, 0.0)
                    arrived = False

        # demo 模式切换状态
        if demo:
            if frame == 300:
                car = make_car(100, 300, 0.0)
                target = tc.MovingTarget(700, 300, bounds=(WIDTH, HEIGHT))
                target.set_pos(800, 300)   # 固定在障碍物(600,380)后方偏右
                target.vx = target.vy = 0.0
                auto_mode = True
            elif frame == 480:
                auto_mode = False  # 切手动

        keys = pygame.key.get_pressed()
        if auto_mode:
            arrived = car.auto_step(dt, target.x, target.y)
            target.step(dt)
        else:
            if demo:
                car.manual_step(dt, forward=1, turn=-0.8)
            else:
                forward = 0
                if keys[pygame.K_UP] or keys[pygame.K_w]:
                    forward = 1
                elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
                    forward = -1
                turn = 0.0
                if keys[pygame.K_LEFT] or keys[pygame.K_a]:
                    turn = 1.0
                elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
                    turn = -1.0
                car.manual_step(dt, forward=forward, turn=turn)
            target.step(dt)

        # 绘制
        screen.fill(BG)
        draw_grid(screen)
        draw_obstacles(screen)
        draw_trail(screen, car)
        target.draw = getattr(target, 'draw', None)
        _draw_target(screen, target, arrived)
        _draw_car(screen, car)
        pygame.draw.line(screen, (100, 100, 100),
                         (car.x, car.y), (target.x, target.y), 1)
        draw_hud(screen, car, target, auto_mode, arrived, font)
        pygame.display.flip()

        # demo 模式：到指定帧就截图
        if demo:
            if frame in frames:
                pygame.image.save(screen, frames[frame])
                print(f"saved: {frames[frame]}")
            if frame > 600:
                running = False

    pygame.quit()


def _draw_car(screen, car):
    size = 16
    th = car.theta
    p1 = (car.x + size * math.cos(th), car.y + size * math.sin(th))
    p2 = (car.x + size * 0.7 * math.cos(th + 2.5),
          car.y + size * 0.7 * math.sin(th + 2.5))
    p3 = (car.x + size * 0.7 * math.cos(th - 2.5),
          car.y + size * 0.7 * math.sin(th - 2.5))
    pygame.draw.polygon(screen, CAR_COLOR, [p1, p2, p3])
    pygame.draw.circle(screen, (200, 200, 200), (int(car.x), int(car.y)), 3)


def _draw_target(screen, target, arrived):
    color = ARRIVED_COLOR if arrived else TARGET_COLOR
    pygame.draw.circle(screen, color, (int(target.x), int(target.y)), 12, 0)
    pygame.draw.circle(screen, color, (int(target.x), int(target.y)), 18, 2)


if __name__ == "__main__":
    run(demo=("--demo" in sys.argv))
