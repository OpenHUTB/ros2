"""
2D 小车目标追踪仿真（Pygame）

- 空格切换：自动追目标 / 手动键盘控制
- 目标自己会随机移动，碰到窗口边界反弹
- 小车带轨迹线
- 场景里有静态圆形障碍物，碰到会被挡住
- 鼠标点一下：在自动模式下设置新目标位置
- R 重置，ESC 退出
- ``--demo``：自动跑三个场景并保存截图
- ``--headless``：不开窗口，跑固定时长并输出 metrics.json，便于回归测试
"""
import argparse
import json
import math
import os
import random
import sys

import pygame

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

# ---------- 控制参数 ----------
MAX_SPEED = 180.0
MAX_STEER = 3.5
KP_TURN = 2.5
ARRIVE_RADIUS = 18.0
CAR_RADIUS = 10.0
MANUAL_SPEED = 160.0
MANUAL_STEER = 3.0
#: 距离目标多远开始线性减速；太大（比如 120）会追不上 60 px/s 的移动目标
SLOWDOWN_DISTANCE = 60.0

#: 固定仿真步长，headless/demo 用它保证可复现
FIXED_DT = 1.0 / 60.0
#: 默认随机种子
DEFAULT_SEED = 42

# 静态障碍物（圆心x, 圆心y, 半径）
OBSTACLES = [
    (300, 200, 35),
    (600, 380, 45),
    (450, 450, 30),
]


def normalize_angle(a):
    """把角度折到 [-pi, pi]（-pi 和 pi 都表示同一方向）。"""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Car:
    def __init__(self, x, y, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.trail = []  # 轨迹
        self.collisions = 0
        # 一次持续贴墙只记一次碰撞：从上一次“自由”进入“被挡住”时 +1
        self._blocked_last = False
        # 撞到障碍物后，保持同一个转向方向一小段时间，避免每帧来回抖
        self._avoid_dir = 0.0
        self._avoid_time = 0.0

    def _note_blocked(self):
        if not self._blocked_last:
            self.collisions += 1
        self._blocked_last = True

    def _note_free(self):
        self._blocked_last = False

    def _collide_obstacle(self, nx, ny):
        """检查新位置是否撞到障碍物，撞到返回 True。"""
        for (ox, oy, r) in OBSTACLES:
            if math.hypot(nx - ox, ny - oy) < r + CAR_RADIUS:
                return True
        return False

    def auto_step(self, dt, tx, ty):
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < ARRIVE_RADIUS:
            self.v = 0.0
            self.omega = 0.0
            self._note_free()
            self.trail.append((self.x, self.y))
            return True

        target_angle = math.atan2(dy, dx)
        angle_err = normalize_angle(target_angle - self.theta)
        self.omega = max(-MAX_STEER, min(MAX_STEER, KP_TURN * angle_err))
        speed_factor = max(0.0, math.cos(angle_err))
        stop_factor = min(1.0, dist / SLOWDOWN_DISTANCE)
        self.v = MAX_SPEED * speed_factor * stop_factor

        # 先算新位置，再检查碰撞
        new_theta = self.theta + self.omega * dt
        new_x = self.x + self.v * math.cos(new_theta) * dt
        new_y = self.y + self.v * math.sin(new_theta) * dt

        if self._collide_obstacle(new_x, new_y):
            # 撞墙：不前进，只原地转，保持一个方向转一小段时间再试
            self.v = 0.0
            self._note_blocked()
            if self._avoid_time <= 0.0:
                self._avoid_dir = 1.0 if angle_err >= 0.0 else -1.0
                self._avoid_time = 0.35
            self.omega = MAX_STEER * self._avoid_dir
            self.theta = normalize_angle(self.theta + self.omega * dt)
            self._avoid_time -= dt
        else:
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
            self._avoid_time = 0.0
            self._note_free()

        self.trail.append((self.x, self.y))
        return False

    def manual_step(self, dt, keys):
        self.v = 0.0
        self.omega = 0.0
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            self.v = MANUAL_SPEED
        elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
            self.v = -MANUAL_SPEED * 0.6
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            self.omega = MANUAL_STEER
        elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            self.omega = -MANUAL_STEER

        new_theta = self.theta + self.omega * dt
        new_x = self.x + self.v * math.cos(new_theta) * dt
        new_y = self.y + self.v * math.sin(new_theta) * dt
        # 位置被挡住时仍然允许原地转向，否则贴墙后无法掉头
        self.theta = new_theta
        if not self._collide_obstacle(new_x, new_y):
            self.x = new_x
            self.y = new_y
            self._note_free()
        else:
            self.v = 0.0
            self._note_blocked()
        self.trail.append((self.x, self.y))

    def draw(self, screen):
        size = 16
        p1 = (self.x + size * math.cos(self.theta),
              self.y + size * math.sin(self.theta))
        p2 = (self.x + size * 0.7 * math.cos(self.theta + 2.5),
              self.y + size * 0.7 * math.sin(self.theta + 2.5))
        p3 = (self.x + size * 0.7 * math.cos(self.theta - 2.5),
              self.y + size * 0.7 * math.sin(self.theta - 2.5))
        pygame.draw.polygon(screen, CAR_COLOR, [p1, p2, p3])
        pygame.draw.circle(screen, (200, 200, 200), (int(self.x), int(self.y)), 3)


class MovingTarget:
    def __init__(self, x, y, rng=None):
        self.x = x
        self.y = y
        rng = rng if rng is not None else random
        angle = rng.uniform(0, 2 * math.pi)
        speed = 60.0
        self.vx = speed * math.cos(angle)
        self.vy = speed * math.sin(angle)

    def step(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        # 边界反弹
        margin = 25
        if self.x < margin or self.x > WIDTH - margin:
            self.vx *= -1
            self.x = max(margin, min(WIDTH - margin, self.x))
        if self.y < margin or self.y > HEIGHT - margin:
            self.vy *= -1
            self.y = max(margin, min(HEIGHT - margin, self.y))

    def set_pos(self, x, y):
        self.x = x
        self.y = y

    def draw(self, screen, arrived):
        color = ARRIVED_COLOR if arrived else TARGET_COLOR
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), 12, 0)
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), 18, 2)


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
        pygame.draw.lines(screen, TRACK_COLOR, False, car.trail, 2)


def draw_hud(screen, car, target, auto_mode, arrived, font):
    mode_str = "AUTO (tracking)" if auto_mode else "MANUAL (WASD/arrows)"
    mode_color = ARRIVED_COLOR if auto_mode else (230, 200, 100)
    lines = [
        f"mode: {mode_str}",
        f"speed: {car.v:6.1f} px/s",
        f"angle: {math.degrees(normalize_angle(car.theta)):7.1f} deg",
        f"dist:  {math.hypot(target.x-car.x, target.y-car.y):6.1f} px",
        f"hits:  {car.collisions}",
        "",
        "[SPACE] switch mode   [mouse] set target   [R] reset   [ESC] quit",
    ]
    for i, line in enumerate(lines):
        c = mode_color if i == 0 else TEXT_COLOR
        surf = font.render(line, True, c)
        screen.blit(surf, (12, 12 + i * 22))


def _empty_metrics(seed, duration):
    return {
        "seed": seed,
        "duration": duration,
        "steps": 0,
        "arrived": False,
        "arrived_once": False,
        "time_to_target": None,
        "mean_distance": 0.0,
        "final_distance": 0.0,
        "path_length": 0.0,
        "collisions": 0,
        "mode": "auto",
    }


def _fmt_metrics(m):
    tt = "n/a" if m["time_to_target"] is None else "%.2fs" % m["time_to_target"]
    return ("steps=%d time=%.2fs arrived_once=%s t_target=%s "
            "mean_dist=%.1fpx final_dist=%.1fpx path=%.1fpx hits=%d"
            % (m["steps"], m["duration"], m["arrived_once"], tt,
               m["mean_distance"], m["final_distance"],
               m["path_length"], m["collisions"]))


def run(demo=False, headless=False, seed=DEFAULT_SEED, duration=20.0,
        outdir="demo_out", save_frame=False, quiet=False):
    """运行仿真。

    参数
    ----
    demo : 跑固定的三个演示场景并保存截图
    headless : 不显示窗口（SDL dummy），跑固定时长后退出
    seed : 随机种子，保证目标初始方向可复现
    duration : headless 模式仿真时长（秒）
    outdir : 截图、metrics.json 的输出目录
    save_frame : headless 模式结束时保存一张 final_frame.png
    quiet : 不打印进度和汇总
    """
    if duration <= 0:
        raise ValueError("duration 必须为正")
    if headless:
        # 必须在 pygame.init()/set_mode() 之前设置
        os.environ["SDL_VIDEODRIVER"] = "dummy"

    random.seed(seed)
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("2D Car Target Tracker - SPACE to switch mode")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    car = Car(WIDTH / 2, HEIGHT / 2, 0.0)
    target = MovingTarget(700, 150)
    auto_mode = True
    arrived = False

    need_outdir = demo or headless or save_frame
    if need_outdir:
        os.makedirs(outdir, exist_ok=True)

    demo_frames = {}
    if demo:
        demo_frames = {
            150: "shot_demo_1_auto_chasing.png",
            430: "shot_demo_2_avoiding.png",
            560: "shot_demo_3_manual.png",
        }

    metrics = _empty_metrics(seed, duration)
    dist_sum = 0.0
    frame = 0
    sim_time = 0.0
    running = True
    total_steps = int(round(duration / FIXED_DT)) if headless and not demo else None

    while running:
        if headless or demo:
            dt = FIXED_DT
        else:
            dt = clock.tick(60) / 1000.0
            if dt <= 0.0:
                dt = FIXED_DT
        frame += 1
        sim_time += dt

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
                    car = Car(WIDTH / 2, HEIGHT / 2, 0.0)
                    arrived = False

        # demo 模式切换状态
        if demo:
            if frame == 300:
                # 让小车从障碍物区域左下方出发，目标在右上方，
                # 轨迹会实际绕过障碍物，而不是走一条空直线。
                car = Car(100, 500, 0.0)
                target = MovingTarget(800, 100)
                auto_mode = True
            elif frame == 480:
                auto_mode = False  # 切手动

        prev_x, prev_y = car.x, car.y
        keys = pygame.key.get_pressed()
        if auto_mode:
            arrived = car.auto_step(dt, target.x, target.y)
            target.step(dt)
        else:
            if demo:
                # demo 模式下模拟：前向 + 右转
                car.v = MANUAL_SPEED
                car.omega = -MANUAL_STEER
                new_theta = car.theta + car.omega * dt
                new_x = car.x + car.v * math.cos(new_theta) * dt
                new_y = car.y + car.v * math.sin(new_theta) * dt
                car.theta = new_theta
                if not car._collide_obstacle(new_x, new_y):
                    car.x = new_x
                    car.y = new_y
                    car._note_free()
                else:
                    car.v = 0.0
                    car._note_blocked()
                car.trail.append((car.x, car.y))
            else:
                car.manual_step(dt, keys)
            target.step(dt)

        # 指标
        dist = math.hypot(target.x - car.x, target.y - car.y)
        dist_sum += dist
        metrics["steps"] = frame
        metrics["duration"] = round(sim_time, 3)
        metrics["arrived"] = bool(arrived)
        if arrived:
            metrics["arrived_once"] = True
            if metrics["time_to_target"] is None:
                metrics["time_to_target"] = round(sim_time, 3)
        metrics["final_distance"] = dist
        metrics["path_length"] += math.hypot(car.x - prev_x, car.y - prev_y)
        metrics["collisions"] = car.collisions
        metrics["mode"] = "auto" if auto_mode else "manual"

        # 绘制
        screen.fill(BG)
        draw_grid(screen)
        draw_obstacles(screen)
        draw_trail(screen, car)
        target.draw(screen, arrived)
        car.draw(screen)
        pygame.draw.line(screen, (100, 100, 100),
                         (car.x, car.y), (target.x, target.y), 1)
        draw_hud(screen, car, target, auto_mode, arrived, font)

        if not headless:
            pygame.display.flip()

        # demo 截图
        if demo and frame in demo_frames:
            path = os.path.join(outdir, demo_frames[frame])
            pygame.image.save(screen, path)
            if not quiet:
                print(f"saved: {path}")

        if demo and frame > 600:
            running = False
        if total_steps is not None and frame >= total_steps:
            running = False

    metrics["mean_distance"] = dist_sum / max(1, frame)
    if headless and save_frame:
        path = os.path.join(outdir, "final_frame.png")
        pygame.image.save(screen, path)
        if not quiet:
            print(f"saved: {path}")
    if headless:
        path = os.path.join(outdir, "metrics.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(metrics, f, ensure_ascii=False, indent=2)
        if not quiet:
            print("metrics: " + _fmt_metrics(metrics))
            print(f"saved: {path}")

    pygame.quit()
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description="2D 小车目标追踪仿真")
    parser.add_argument("--demo", action="store_true",
                        help="跑三个演示场景并保存截图")
    parser.add_argument("--headless", action="store_true",
                        help="不显示窗口，跑固定时长并输出 metrics.json")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED,
                        help="随机种子（默认 %d）" % DEFAULT_SEED)
    parser.add_argument("--duration", type=float, default=20.0,
                        help="headless 模式仿真时长，单位秒（默认 20）")
    parser.add_argument("--outdir", default="demo_out",
                        help="截图和 metrics.json 的输出目录（默认 demo_out）")
    parser.add_argument("--save-frame", action="store_true",
                        help="headless 模式结束时保存 final_frame.png")
    parser.add_argument("--quiet", action="store_true",
                        help="不打印进度和汇总")
    args = parser.parse_args(argv)
    run(demo=args.demo, headless=args.headless, seed=args.seed,
        duration=args.duration, outdir=args.outdir,
        save_frame=args.save_frame, quiet=args.quiet)
    return 0


if __name__ == "__main__":
    sys.exit(main())
