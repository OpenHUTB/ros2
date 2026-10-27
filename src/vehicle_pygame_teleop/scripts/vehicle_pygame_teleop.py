#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vehicle_pygame_teleop.py

基于 pygame 的车辆键盘遥操作节点：
1. 以 30Hz 向 /cmd_vel 发布 geometry_msgs/Twist 速度指令；
2. 内置俯视二维驾驶模拟：车辆随速度/转向实时运动，无需外部仿真器即可验证控制效果。

按键：
    W / 方向键上：油门加速
    S / 方向键下：前进时刹车，静止时倒车
    A / 方向键左：左转    D / 方向键右：右转
    空格：手刹急停    ESC 或关闭窗口：发布零速度后退出
"""

import math

import pygame
import rospy
from geometry_msgs.msg import Twist

# ---------------- 画面与仿真参数 ----------------
WINDOW_W, WINDOW_H = 800, 600
FPS = 30
PX_PER_M = 22.0              # 每米对应的像素数
CAR_L, CAR_W = 40, 22         # 车身像素尺寸（长、宽）
MAX_SPEED = 1.5               # 最大前进速度 m/s
MAX_REVERSE = 0.8             # 最大倒车速度 m/s
ACCEL = 2.2                   # 油门加速度 m/s^2
BRAKE = 4.5                   # 刹车减速度 m/s^2
HANDBRAKE = 8.0               # 手刹减速度 m/s^2
DRAG = 1.2                    # 松开按键后的滑行阻力减速度 m/s^2
MAX_YAW_RATE = 1.4            # 最大转向角速度 rad/s
YAW_RESPONSE = 8.0            # 转向响应速度 rad/s^2

# ---------------- 颜色 ----------------
COLOR_BG = (18, 20, 26)
COLOR_GRID_MINOR = (32, 36, 46)
COLOR_GRID_MAJOR = (48, 54, 68)
COLOR_ORIGIN = (90, 100, 120)
COLOR_CAR_BODY = (72, 160, 255)
COLOR_CAR_FRONT = (255, 200, 80)
COLOR_WHEEL = (220, 220, 230)
COLOR_TEXT = (230, 235, 245)


class TeleopCar:
    def __init__(self):
        rospy.init_node('vehicle_pygame_teleop', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        pygame.init()
        pygame.display.set_caption("Vehicle Teleop - built-in 2D driving simulation")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        self.clock = pygame.time.Clock()
        try:
            self.font = pygame.font.SysFont("dejavusans,arial", 16)
            self.font_bold = pygame.font.SysFont("dejavusans,arial", 17, bold=True)
        except Exception:
            # 系统字体枚举失败时回退到 pygame 内置默认字体，保证节点可用
            self.font = pygame.font.Font(None, 22)
            self.font_bold = pygame.font.Font(None, 24)

        # 车辆运动学状态（世界坐标，单位 m / rad）
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0

        rospy.loginfo("车辆键盘遥操作节点已启动（内置二维模拟）")
        rospy.loginfo("W/↑加速  S/↓刹车/倒车  A/←左转  D/→右转  空格手刹  ESC退出")
        # 注册 rospy 关停回调：Ctrl+C 等外部关停时也要发布零速度，
        # 否则最后一条 /cmd_vel 会停留在退出前的速度上
        rospy.on_shutdown(self.on_ros_shutdown)

    def publish_zero(self, wait=True):
        """发布零速度指令并短暂等待，确保停车指令可靠送达。

        wait=False 用于 rospy 关停回调：关停阶段 rospy.sleep 会抛出
        ROSInterruptException，不能再阻塞等待。
        """
        twist = Twist()
        self.pub.publish(twist)
        self.pub.publish(twist)
        if wait:
            rospy.sleep(0.1)

    def on_ros_shutdown(self):
        """rospy 关停回调（Ctrl+C / rosnode kill 等）：保证最后一条指令为零速度"""
        self.publish_zero(wait=False)

    def update_physics(self, dt, keys):
        """根据按键更新车辆速度、航向与位置（差速底盘模型）"""
        if keys[pygame.K_SPACE]:
            # 手刹：快速减速到零
            if self.speed > 0:
                self.speed = max(0.0, self.speed - HANDBRAKE * dt)
            elif self.speed < 0:
                self.speed = min(0.0, self.speed + HANDBRAKE * dt)
            target_yaw = 0.0
        else:
            # 纵向控制
            if keys[pygame.K_UP] or keys[pygame.K_w]:
                if self.speed < 0:
                    self.speed = min(0.0, self.speed + BRAKE * dt)   # 倒车中踩油门先制动
                else:
                    self.speed = min(MAX_SPEED, self.speed + ACCEL * dt)
            elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
                if self.speed > 0:
                    self.speed = max(0.0, self.speed - BRAKE * dt)   # 前进中踩刹车
                else:
                    self.speed = max(-MAX_REVERSE, self.speed - ACCEL * dt)
            else:
                # 无按键时滑行阻力自然减速
                if self.speed > 0:
                    self.speed = max(0.0, self.speed - DRAG * dt)
                elif self.speed < 0:
                    self.speed = min(0.0, self.speed + DRAG * dt)

            # 横向（转向）控制
            target_yaw = 0.0
            if keys[pygame.K_LEFT] or keys[pygame.K_a]:
                target_yaw = MAX_YAW_RATE
            elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
                target_yaw = -MAX_YAW_RATE

        # 倒车时转向效果反向（差速底盘特性）
        if self.speed < -0.02:
            target_yaw = -target_yaw

        # 转向响应平滑
        if self.yaw_rate < target_yaw:
            self.yaw_rate = min(target_yaw, self.yaw_rate + YAW_RESPONSE * dt)
        else:
            self.yaw_rate = max(target_yaw, self.yaw_rate - YAW_RESPONSE * dt)

        # 运动学积分
        self.yaw += self.yaw_rate * dt
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt

    def world_to_screen(self, wx, wy):
        """世界坐标（米，y 轴向北）转屏幕坐标（像素，y 轴向下）"""
        sx = WINDOW_W / 2.0 + (wx - self.x) * PX_PER_M
        sy = WINDOW_H / 2.0 - (wy - self.y) * PX_PER_M
        return sx, sy

    def draw_grid(self):
        """绘制跟随车辆的网格地图"""
        self.screen.fill(COLOR_BG)
        cx, cy = WINDOW_W / 2.0, WINDOW_H / 2.0
        view_m = int(max(WINDOW_W, WINDOW_H) / PX_PER_M / 2) + 2
        for i in range(-view_m, view_m + 1):
            major = i % 5 == 0
            color = COLOR_GRID_MAJOR if major else COLOR_GRID_MINOR
            width = 2 if major else 1
            # 竖直线
            sx = cx + i * PX_PER_M
            pygame.draw.line(self.screen, color, (sx, 0), (sx, WINDOW_H), width)
            # 水平线
            sy = cy - i * PX_PER_M
            pygame.draw.line(self.screen, color, (0, sy), (WINDOW_W, sy), width)
        # 世界原点标记
        zx, zy = self.world_to_screen(0, 0)
        if -10 <= zx <= WINDOW_W + 10 and -10 <= zy <= WINDOW_H + 10:
            pygame.draw.circle(self.screen, COLOR_ORIGIN, (int(zx), int(zy)), 5)

    def draw_car(self):
        """在屏幕中心绘制随航向旋转的车辆（车头朝右为 0 航向）"""
        surf = pygame.Surface((CAR_L + 12, CAR_W + 12), pygame.SRCALPHA)
        # 四个车轮
        pygame.draw.rect(surf, COLOR_WHEEL, (6 + 2, 3, 10, 5))
        pygame.draw.rect(surf, COLOR_WHEEL, (6 + 2, 6 + CAR_W - 2, 10, 5))
        pygame.draw.rect(surf, COLOR_WHEEL, (6 + CAR_L - 12, 3, 10, 5))
        pygame.draw.rect(surf, COLOR_WHEEL, (6 + CAR_L - 12, 6 + CAR_W - 2, 10, 5))
        # 车身（不使用 border_radius 关键字参数：Ubuntu 20.04 apt 源的 pygame
        # 1.9.6 不支持它，会报 TypeError: rect() takes no keyword arguments）
        pygame.draw.rect(surf, COLOR_CAR_BODY, (6, 6, CAR_L, CAR_W))
        # 车头标识（前挡风玻璃）
        pygame.draw.rect(surf, COLOR_CAR_FRONT,
                         (6 + CAR_L - 14, 6 + 4, 8, CAR_W - 8))
        rotated = pygame.transform.rotate(surf, math.degrees(self.yaw))
        rect = rotated.get_rect(center=(WINDOW_W / 2.0, WINDOW_H / 2.0))
        self.screen.blit(rotated, rect)

    def draw_hud(self):
        """绘制左上角状态与按键提示面板"""
        panel = pygame.Surface((330, 132), pygame.SRCALPHA)
        panel.fill((10, 12, 16, 210))
        self.screen.blit(panel, (10, 10))
        lines = [
            ("Topic /cmd_vel  (geometry_msgs/Twist @30Hz)", True),
            ("linear.x : %+.2f m/s" % self.speed, False),
            ("angular.z: %+.2f rad/s" % self.yaw_rate, False),
            ("W/Up gas    S/Down brake/reverse", False),
            ("A/Left left    D/Right right", False),
            ("Space handbrake    ESC quit", False),
        ]
        for i, (text, bold) in enumerate(lines):
            font = self.font_bold if bold else self.font
            self.screen.blit(font.render(text, True, COLOR_TEXT), (20, 20 + i * 19))

    def render(self):
        self.draw_grid()
        self.draw_car()
        self.draw_hud()
        pygame.display.flip()

    def shutdown(self):
        self.publish_zero()
        pygame.quit()

    def run(self):
        while not rospy.is_shutdown():
            dt = min(self.clock.tick(FPS) / 1000.0, 0.1)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.shutdown()
                    return
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    self.shutdown()
                    return

            keys = pygame.key.get_pressed()
            self.update_physics(dt, keys)

            twist = Twist()
            twist.linear.x = self.speed
            twist.angular.z = self.yaw_rate
            self.pub.publish(twist)

            self.render()


def main():
    teleop = None
    try:
        teleop = TeleopCar()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 非关停的退出路径（如窗口关闭）兜底再补发一次零速度；
        # Ctrl+C 等关停场景由 on_shutdown 回调负责（关停阶段不能再调 rospy.sleep）
        if teleop is not None and not rospy.is_shutdown():
            teleop.publish_zero()
        pygame.quit()


if __name__ == '__main__':
    main()
