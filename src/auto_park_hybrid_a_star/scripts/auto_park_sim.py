#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""auto_park_sim.py

基于 Pygame 的垂直自动泊车仿真（无 ROS 也能跑）。

场景：路边一排垂直车位，目标车位空着，两侧停着障碍车；
     小车从道路上出发，按 [空格] 触发 Hybrid A* 规划并自动泊入。

依赖：pygame（pip install pygame）
运行：python auto_park_sim.py
按键：空格 = 开始规划并泊车；R = 重置；ESC = 退出
"""

import math
import sys

import pygame

from collision_check import VehicleFootprint, check_collision, vehicle_corners
from hybrid_a_star import HybridAStar
from tracker import DiffRCSim, PurePursuit

# ---------------- 画面参数 ----------------
WINDOW_W, WINDOW_H = 1100, 750
PX_PER_M = 90
FPS = 30

# ---------------- 场景参数（米）----------------
CAR = VehicleFootprint(length=4.4, width=1.9, wheel_base=2.7)
# 车位几何
SLOT_W = 2.6          # 车位宽
SLOT_D = 5.2          # 车位深
SLOT_Y0 = 0.0         # 车位入口（道路边缘）
# 三个车位：左邻位（障碍）、目标位、右邻位（障碍）
LEFT_SLOT_X0 = -5.5
GOAL_SLOT_X0 = -2.9   # 目标车位左边界
RIGHT_SLOT_X0 = -0.3
GOAL_CX = GOAL_SLOT_X0 + SLOT_W / 2.0
GOAL_CY = SLOT_Y0 + SLOT_D / 2.0

# 小车起点：道路上，车头朝 +x
START = (2.5, -2.2, 0.0)
# 目标位姿：车位中央，车头朝 +y（即 yaw = pi/2）
GOAL = (GOAL_CX, GOAL_CY, math.pi / 2.0)

# 障碍车（AABB 中心 / 宽 / 高）
OBSTACLES = [
    # 左邻位停车（车长沿 y 方向，所以 AABB 宽=车宽，高=车长）
    (LEFT_SLOT_X0 + SLOT_W / 2.0, SLOT_Y0 + SLOT_D / 2.0, CAR.width, CAR.length),
    # 右邻位停车
    (RIGHT_SLOT_X0 + SLOT_W / 2.0, SLOT_Y0 + SLOT_D / 2.0, CAR.width, CAR.length),
]

# ---------------- 颜色 ----------------
C_BG = (24, 26, 32)
C_ROAD = (48, 52, 60)
C_SLOT_LINE = (180, 190, 200)
C_GOAL_SLOT = (80, 200, 120)
C_OBS_CAR = (90, 96, 110)
C_CAR = (70, 150, 255)
C_CAR_FRONT = (255, 200, 80)
C_PATH_FWD = (120, 220, 140)
C_PATH_REV = (255, 120, 120)
C_TEXT = (235, 240, 250)


class ParkingSim:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Auto Parking - Hybrid A* + Reeds-Shepp")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("microsoftyahei,arial", 16)
        self.font_big = pygame.font.SysFont("microsoftyahei,arial", 22, bold=True)

        self.car = DiffRCSim(CAR)
        self.reset()

    def reset(self):
        self.car.reset(*START)
        self.path = None           # 规划好的参考路径
        self.tracker = None
        self.state = "idle"        # idle / planning / tracking / done / failed
        self.message = "按 [空格] 开始自动泊车"

    # ---------- 坐标变换 ----------
    def w2s(self, wx, wy):
        """世界坐标（米，y 向上）→ 屏幕像素（跟随小车）"""
        sx = WINDOW_W / 2.0 + (wx - self.car.x) * PX_PER_M
        sy = WINDOW_H / 2.0 - (wy - self.car.y) * PX_PER_M
        return sx, sy

    # ---------- 绘制 ----------
    def draw_road(self):
        # 路面
        road_rect = pygame.Rect(0, 0, WINDOW_W, WINDOW_H)
        pygame.draw.rect(self.screen, C_BG, road_rect)
        # 道路区（y 从 -4 到 0）
        for wy in (-4.0, -2.0, 0.0):
            sx, sy = self.w2s(-20, wy)
            sx2, sy2 = self.w2s(20, wy)
            pygame.draw.line(self.screen, C_ROAD, (sx, sy), (sx2, sy), 2)
        # 车位线
        for x0 in (LEFT_SLOT_X0, GOAL_SLOT_X0, RIGHT_SLOT_X0):
            # 入口横线
            p1 = self.w2s(x0, SLOT_Y0)
            p2 = self.w2s(x0 + SLOT_W, SLOT_Y0)
            pygame.draw.line(self.screen, C_SLOT_LINE, p1, p2, 2)
            # 两侧竖线
            for edge_x in (x0, x0 + SLOT_W):
                p1 = self.w2s(edge_x, SLOT_Y0)
                p2 = self.w2s(edge_x, SLOT_Y0 + SLOT_D)
                pygame.draw.line(self.screen, C_SLOT_LINE, p1, p2, 2)
        # 目标车位高亮
        gx1 = self.w2s(GOAL_SLOT_X0, SLOT_Y0)
        gx2 = self.w2s(GOAL_SLOT_X0 + SLOT_W, SLOT_Y0 + SLOT_D)
        rect = pygame.Rect(min(gx1[0], gx2[0]), min(gx1[1], gx2[1]),
                           abs(gx2[0] - gx1[0]), abs(gx2[1] - gx1[1]))
        pygame.draw.rect(self.screen, (40, 80, 55), rect)

    def draw_obstacles(self):
        for (cx, cy, w, h) in OBSTACLES:
            # 障碍车是矩形，车头朝 +y
            corners_local = [(-w/2, -h/2), (-w/2, h/2), (w/2, h/2), (w/2, -h/2)]
            screen_pts = [self.w2s(cx + lx, cy + ly) for lx, ly in corners_local]
            pygame.draw.polygon(self.screen, C_OBS_CAR, screen_pts)

    def draw_path(self):
        if not self.path:
            return
        for i in range(len(self.path) - 1):
            x1, y1, _, g1 = self.path[i]
            x2, y2, _, _ = self.path[i + 1]
            color = C_PATH_REV if g1 < 0 else C_PATH_FWD
            p1 = self.w2s(x1, y1)
            p2 = self.w2s(x2, y2)
            pygame.draw.line(self.screen, color, p1, p2, 3)

    def draw_car(self):
        # 用车辆实际角点画旋转矩形
        corners_world = vehicle_corners(self.car.x, self.car.y, self.car.yaw, CAR)
        screen_pts = [self.w2s(x, y) for x, y in corners_world]
        pygame.draw.polygon(self.screen, C_CAR, screen_pts)
        # 车头标识
        fx = self.car.x + CAR.wheel_base * math.cos(self.car.yaw)
        fy = self.car.y + CAR.wheel_base * math.sin(self.car.yaw)
        p = self.w2s(fx, fy)
        pygame.draw.circle(self.screen, C_CAR_FRONT, (int(p[0]), int(p[1])), 5)

    def draw_hud(self):
        lines = [
            ("Auto Parking: Hybrid A* + Reeds-Shepp", True),
            ("state: %s" % self.state, False),
            ("x=%.2f  y=%.2f  yaw=%.0f deg" %
             (self.car.x, self.car.y, math.degrees(self.car.yaw)), False),
            ("v=%.2f m/s  steer=%.0f deg" %
             (self.car.v, math.degrees(self.car.steer)), False),
            ("", False),
            ("[SPACE] plan & park   [R] reset   [ESC] quit", False),
            (self.message, False),
        ]
        for i, (text, bold) in enumerate(lines):
            font = self.font_big if bold else self.font
            surf = font.render(text, True, C_TEXT)
            self.screen.blit(surf, (15, 15 + i * 26))

    # ---------- 规划 ----------
    def plan(self):
        self.state = "planning"
        self.message = "Hybrid A* 规划中..."
        pygame.display.flip()
        planner = HybridAStar(CAR, OBSTACLES, step=1.2, max_iter=12000)
        path = planner.plan(START, GOAL, goal_tol_xy=0.35,
                            goal_tol_yaw=math.radians(20))
        if path is None:
            self.state = "failed"
            self.message = "规划失败！按 R 重置后重试"
            return
        from tracker import _densify
        path = _densify(path, 0.1)
        self.path = path
        self.tracker = PurePursuit(path, CAR, lookahead=1.0, speed=0.7)
        self.state = "tracking"
        self.message = "跟踪路径中..."

    # ---------- 主循环 ----------
    def run(self):
        running = True
        while running:
            dt = self.clock.tick(FPS) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE and self.state == "idle":
                        self.plan()
                    elif event.key == pygame.K_r:
                        self.reset()

            # 跟踪更新
            if self.state == "tracking" and self.tracker is not None:
                v_cmd, s_cmd, info = self.tracker.run(self.car, dt)
                self.car.step(v_cmd, s_cmd, dt)
                if info.get("at_goal"):
                    self.state = "done"
                    err = info.get("err", 0.0)
                    self.message = "泊车完成！横向误差 %.2f m" % err

            # 渲染
            self.draw_road()
            self.draw_obstacles()
            self.draw_path()
            self.draw_car()
            self.draw_hud()
            pygame.display.flip()

        pygame.quit()


if __name__ == "__main__":
    # 检查 pygame
    try:
        import pygame  # noqa
    except ImportError:
        print("请先安装 pygame:  pip install pygame")
        sys.exit(1)
    ParkingSim().run()
