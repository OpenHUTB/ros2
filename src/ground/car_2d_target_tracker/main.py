"""
2D 小车目标追踪仿真（Pygame）
- 空格切换：自动追目标 / 手动键盘控制
- 目标自己会随机移动
- 小车带轨迹线
- 场景里有静态障碍物，碰到会被挡住
- 鼠标点一下：在自动模式下设置新目标位置
- R 重置，ESC 退出
- 加 --demo 参数：自动跑几个场景并保存截图后退出
"""
import math
import sys
import random
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

# 静态障碍物（圆心x, 圆心y, 半径）
OBSTACLES = [
    (300, 200, 35),
    (600, 380, 45),
    (450, 450, 30),
]


def normalize_angle(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class Car:
    def __init__(self, x, y, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.trail = []  # 轨迹

    def _collide_obstacle(self, nx, ny):
        """检查新位置是否撞到障碍物，撞到返回 True"""
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
            self.trail.append((self.x, self.y))
            return True

        target_angle = math.atan2(dy, dx)
        angle_err = normalize_angle(target_angle - self.theta)
        self.omega = max(-MAX_STEER, min(MAX_STEER, KP_TURN * angle_err))
        speed_factor = max(0.0, math.cos(angle_err))
        stop_factor = min(1.0, dist / 120.0)
        self.v = MAX_SPEED * speed_factor * stop_factor

        # 先算新位置，再检查碰撞
        new_theta = self.theta + self.omega * dt
        new_x = self.x + self.v * math.cos(new_theta) * dt
        new_y = self.y + self.v * math.sin(new_theta) * dt

        if self._collide_obstacle(new_x, new_y):
            # 撞墙：不前进，但继续转向（让小车绕着障碍物转）
            self.v = 0.0
            self.omega = math.copysign(MAX_STEER, self.omega if self.omega != 0 else 1.0)
        else:
            self.x = new_x
            self.y = new_y
            self.theta = new_theta

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
        if not self._collide_obstacle(new_x, new_y):
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
        else:
            self.v = 0.0
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
    def __init__(self, x, y):
        self.x = x
        self.y = y
        angle = random.uniform(0, 2 * math.pi)
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
        "",
        "[SPACE] switch mode   [mouse] set target   [R] reset   [ESC] quit",
    ]
    for i, line in enumerate(lines):
        c = mode_color if i == 0 else TEXT_COLOR
        surf = font.render(line, True, c)
        screen.blit(surf, (12, 12 + i * 22))


def run(demo=False):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("2D Car Target Tracker - SPACE to switch mode")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    car = Car(WIDTH / 2, HEIGHT / 2, 0.0)
    target = MovingTarget(700, 150)
    auto_mode = True
    arrived = False

    frames = {}  # demo 模式：frame -> 保存文件名
    if demo:
        random.seed(42)
        frames[150] = "shot_demo_1_auto_chasing.png"
        # frame 300 重置到绕障碍物场景，frame 430 截图（这时已有轨迹）
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
                    car = Car(WIDTH / 2, HEIGHT / 2, 0.0)
                    arrived = False

        # demo 模式切换状态
        if demo:
            if frame == 300:
                car = Car(100, 300, 0.0)
                target = MovingTarget(800, 300)
                auto_mode = True
            elif frame == 480:
                auto_mode = False  # 切手动

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
                if not car._collide_obstacle(new_x, new_y):
                    car.x = new_x; car.y = new_y; car.theta = new_theta
                car.trail.append((car.x, car.y))
            else:
                car.manual_step(dt, keys)
            target.step(dt)

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
        pygame.display.flip()

        # demo 模式：到指定帧就截图
        if demo:
            if frame in frames:
                pygame.image.save(screen, frames[frame])
                print(f"saved: {frames[frame]}")
            if frame > 600:
                running = False

    pygame.quit()


if __name__ == "__main__":
    run(demo=("--demo" in sys.argv))
