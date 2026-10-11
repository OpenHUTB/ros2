#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车辆键盘遥操作节点（ROS 1 Noetic）。

键盘 → 指令平滑（30Hz）→ /carla/ego_vehicle/twist → carla_twist_to_control
     → /carla/ego_vehicle/vehicle_control_cmd → CARLA 0.9.16 中的自车。

物理仿真由 CARLA 提供，本节点只生成控制指令（不含任何仿真）。

按键：W/↑ 加速，S/↓ 刹车（静止后倒车），A/← 左转，D/→ 右转，
      空格 急停（零 Twist 在 CARLA 侧为全力刹车），ESC/关窗/Ctrl+C 退出并发布零速度。
话题：<~twist_topic>（默认 /carla/ego_vehicle/twist，geometry_msgs/Twist）
"""

import pygame
import rospy
from geometry_msgs.msg import Twist

# 指令参数（carla_twist_to_control：10 m/s 对应满油门，angular.z 即前轮转角）
FPS = 30
WINDOW_W, WINDOW_H = 560, 190
MAX_SPEED, MAX_REVERSE = 10.0, 5.0                    # 前进 / 倒车指令上限 (m/s)
MAX_STEER = 0.6                                       # 转向指令上限 (rad)
ACCEL, BRAKE, HANDBRAKE, DRAG = 4.0, 8.0, 12.0, 3.0   # 指令变化率 (m/s^2)
STEER_RATE = 3.0                                      # 转向指令响应速率 (rad/s)

COLOR_BG = (18, 20, 26)
COLOR_PANEL = (28, 32, 40)
COLOR_TEXT = (230, 235, 245)
COLOR_VALUE = (255, 200, 80)
COLOR_HINT = (140, 150, 170)


class TeleopCar:
    def __init__(self):
        rospy.init_node('vehicle_pygame_teleop', anonymous=True)
        self.twist_topic = rospy.get_param('~twist_topic', '/carla/ego_vehicle/twist')
        self.pub = rospy.Publisher(self.twist_topic, Twist, queue_size=10)

        pygame.init()
        pygame.display.set_caption("Vehicle Teleop -> CARLA 0.9.16")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        self.clock = pygame.time.Clock()
        try:
            self.font = pygame.font.SysFont("dejavusans,arial", 16)
            self.font_bold = pygame.font.SysFont("dejavusans,arial", 17, bold=True)
        except Exception:
            # 字体枚举失败时回退到内置字体（兼容 Ubuntu 20.04 apt 的 pygame 1.9.6）
            self.font = pygame.font.Font(None, 22)
            self.font_bold = pygame.font.Font(None, 24)

        self.speed = 0.0    # 速度指令，负值为倒车 (m/s)
        self.steer = 0.0    # 转向指令 (rad)

        # 关停时（Ctrl+C 等）保证最后一条指令为零速度
        rospy.on_shutdown(self.on_ros_shutdown)

    def publish_zero(self, wait=True):
        """发布零速度指令；wait=False 用于关停回调（此时不能再调 rospy.sleep）。"""
        twist = Twist()
        self.pub.publish(twist)
        self.pub.publish(twist)
        if wait:
            rospy.sleep(0.1)

    def on_ros_shutdown(self):
        self.publish_zero(wait=False)

    def update_command(self, dt, keys):
        """按按键更新速度/转向指令（带平滑；车辆运动本身由 CARLA 仿真）。"""
        if keys[pygame.K_SPACE]:
            # 急停
            if self.speed > 0:
                self.speed = max(0.0, self.speed - HANDBRAKE * dt)
            else:
                self.speed = min(0.0, self.speed + HANDBRAKE * dt)
            target_steer = 0.0
        else:
            if keys[pygame.K_UP] or keys[pygame.K_w]:
                # 倒车中踩油门先制动
                step = BRAKE if self.speed < 0 else ACCEL
                self.speed = min(MAX_SPEED, self.speed + step * dt)
            elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
                # 前进中踩刹车，静止后倒车
                step = -BRAKE if self.speed > 0 else -ACCEL
                self.speed = max(-MAX_REVERSE, self.speed + step * dt)
            else:
                # 无按键时指令自然回落
                step = -DRAG if self.speed > 0 else DRAG
                self.speed = max(0.0, self.speed + step * dt) if step < 0 \
                    else min(0.0, self.speed + step * dt)

            target_steer = MAX_STEER if (keys[pygame.K_LEFT] or keys[pygame.K_a]) else (
                -MAX_STEER if (keys[pygame.K_RIGHT] or keys[pygame.K_d]) else 0.0)

        if self.steer < target_steer:
            self.steer = min(target_steer, self.steer + STEER_RATE * dt)
        else:
            self.steer = max(target_steer, self.steer - STEER_RATE * dt)

    def draw_panel(self):
        """状态面板：当前指令、话题与按键提示（本节点无仿真画面）。"""
        self.screen.fill(COLOR_BG)
        pygame.draw.rect(self.screen, COLOR_PANEL,
                         pygame.Rect(10, 10, WINDOW_W - 20, WINDOW_H - 20))
        self.screen.blit(self.font_bold.render(
            "Vehicle Teleop -> CARLA 0.9.16", True, COLOR_TEXT), (24, 22))
        self.screen.blit(self.font.render(
            "topic: %s  (Twist @30Hz)" % self.twist_topic, True, COLOR_HINT), (24, 46))
        self.screen.blit(self.font.render(
            "linear.x : %+.2f m/s" % self.speed, True, COLOR_VALUE), (24, 72))
        self.screen.blit(self.font.render(
            "angular.z: %+.2f rad" % self.steer, True, COLOR_VALUE), (24, 94))
        self.screen.blit(self.font.render(
            "W/Up gas   S/Down brake-reverse   A/Left   D/Right", True, COLOR_TEXT), (24, 124))
        self.screen.blit(self.font.render(
            "Space emergency stop    ESC quit (publish zero)", True, COLOR_HINT), (24, 146))

    def run(self):
        while not rospy.is_shutdown():
            dt = min(self.clock.tick(FPS) / 1000.0, 0.1)

            for event in pygame.event.get():
                if event.type == pygame.QUIT or (
                        event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    self.publish_zero()
                    pygame.quit()
                    return

            self.update_command(dt, pygame.key.get_pressed())

            twist = Twist()
            twist.linear.x = self.speed
            twist.angular.z = self.steer
            self.pub.publish(twist)

            self.draw_panel()
            pygame.display.flip()


def main():
    teleop = None
    try:
        teleop = TeleopCar()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 非关停退出路径的兜底；关停路径由 on_shutdown 回调负责
        if teleop is not None and not rospy.is_shutdown():
            teleop.publish_zero()
        pygame.quit()


if __name__ == '__main__':
    main()
