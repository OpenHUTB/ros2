#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vehicle_pygame_teleop.py

基于 pygame 的车辆键盘遥操作节点：
1. 按 ``rate/publish_hz``（默认 30Hz）向 ``topic/cmd_vel`` 发布 geometry_msgs/Twist；
2. 内置俯视二维驾驶模拟：车辆随速度/转向实时运动，无需外部仿真器即可验证控制效果。

按键：
    W / 方向键上：油门加速
    S / 方向键下：前进时刹车，静止时倒车
    A / 方向键左：左转    D / 方向键右：右转
    空格：手刹急停    ESC 或关闭窗口：发布零速度后退出

参数：全部可调项由 launch/``config/vehicle_pygame_teleop.yaml`` 提供
（``max_speed``/``accel``/``brake``/``handbrake``/``drag``/``max_yaw_rate``/
``yaw_response``/``reverse_steer_invert``/``publish_hz``/``cmd_vel_topic``），
每项默认值都等于改造前的硬编码常量，因此不传参数时行为不变。

分层：本文件只负责 pygame 渲染与 rospy 收发；按键解释与运动学积分在
``teleop_core``（不依赖 pygame / ROS），因此核心逻辑可以做离线回归测试：
    python3 tests/test_vehicle_pygame_teleop_local.py
"""

from __future__ import division, print_function

import math
import os
import sys

# 允许直接 rosrun / 直接执行本脚本时也能导入同目录的 teleop_core；
# 同时补 realpath，兼容 catkin 把 __file__ 指到 lib/<pkg>/ 的 relay 布局。
for _path in (os.path.abspath(__file__), os.path.realpath(__file__)):
    _dir = os.path.dirname(_path)
    if _dir not in sys.path:
        sys.path.insert(0, _dir)

import pygame
import rospy
from geometry_msgs.msg import Twist

import teleop_core as core

# ---------------- 画面参数（仅渲染用，与车辆物理参数无关） ----------------
WINDOW_W, WINDOW_H = 800, 600
PX_PER_M = 22.0               # 每米对应的像素数
CAR_L, CAR_W = 40, 22         # 车身像素尺寸（长、宽）

# ---------------- 颜色 ----------------
COLOR_BG = (18, 20, 26)
COLOR_GRID_MINOR = (32, 36, 46)
COLOR_GRID_MAJOR = (48, 54, 68)
COLOR_ORIGIN = (90, 100, 120)
COLOR_CAR_BODY = (72, 160, 255)
COLOR_CAR_FRONT = (255, 200, 80)
COLOR_WHEEL = (220, 220, 230)
COLOR_TEXT = (230, 235, 245)

# teleop_core.KEY_BINDINGS 里的键名 → pygame 键码。
# 新增/修改按键时只改 teleop_core 的 KEY_BINDINGS 与这里的对应项即可。
_KEYCODE = {
    "w": pygame.K_w, "up": pygame.K_UP,
    "s": pygame.K_s, "down": pygame.K_DOWN,
    "a": pygame.K_a, "left": pygame.K_LEFT,
    "d": pygame.K_d, "right": pygame.K_RIGHT,
    "space": pygame.K_SPACE,
}


class TeleopCar(object):
    def __init__(self):
        rospy.init_node('vehicle_pygame_teleop', anonymous=True)

        # 参数全部从 ROS 参数服务器读取；launch 未提供时用与原来相同的默认值
        self.params = core.TeleopParams.from_params(rospy.get_param)
        # pygame.Clock.tick 需要整数帧率，四舍五入后至少 1Hz
        self._fps = max(1, int(round(self.params.publish_hz)))

        self.pub = rospy.Publisher(self.params.cmd_vel_topic, Twist, queue_size=10)

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
        self.state = core.VehicleState()

        rospy.loginfo("车辆键盘遥操作节点已启动（内置二维模拟）")
        rospy.loginfo("参数：topic=%s  rate=%.1fHz  max_speed=%.2fm/s  accel=%.2fm/s^2"
                      "  brake=%.2fm/s^2  倒车转向反向=%s",
                      self.params.cmd_vel_topic, self.params.publish_hz,
                      self.params.max_speed, self.params.accel, self.params.brake,
                      self.params.reverse_steer_invert)
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

    def read_control(self, keys):
        """把 pygame 的按键状态翻译成与渲染无关的控制意图（逻辑在 teleop_core）"""
        return core.control_from_predicate(lambda name: keys[_KEYCODE[name]])

    def update_physics(self, dt, keys):
        """根据按键更新车辆速度、航向与位置（差速底盘模型）"""
        control = self.read_control(keys)
        core.step_vehicle(self.state, control, dt, self.params)

    def world_to_screen(self, wx, wy):
        """世界坐标（米，y 轴向北）转屏幕坐标（像素，y 轴向下）"""
        sx = WINDOW_W / 2.0 + (wx - self.state.x) * PX_PER_M
        sy = WINDOW_H / 2.0 - (wy - self.state.y) * PX_PER_M
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
        rotated = pygame.transform.rotate(surf, math.degrees(self.state.yaw))
        rect = rotated.get_rect(center=(WINDOW_W / 2.0, WINDOW_H / 2.0))
        self.screen.blit(rotated, rect)

    def draw_hud(self):
        """绘制左上角状态与按键提示面板"""
        panel = pygame.Surface((330, 132), pygame.SRCALPHA)
        panel.fill((10, 12, 16, 210))
        self.screen.blit(panel, (10, 10))
        # 话题名与频率按实际参数显示（原来写死 /cmd_vel 与 30Hz）
        lines = [
            ("Topic %s  (geometry_msgs/Twist @%dHz)"
             % (self.params.cmd_vel_topic, self._fps), True),
            ("linear.x : %+.2f m/s" % self.state.speed, False),
            ("angular.z: %+.2f rad/s" % self.state.yaw_rate, False),
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
            # dt 上限 0.1s：窗口拖动/系统卡顿后不让一帧积分过大而“瞬移”
            dt = min(self.clock.tick(self._fps) / 1000.0, 0.1)

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
            twist.linear.x, twist.angular.z = core.twist_fields(self.state)
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
