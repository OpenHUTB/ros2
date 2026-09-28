#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥控节点：把按键映射为车辆控制意图并发布，不直接触碰 CARLA。

ROS 2 接口（Humble）：
  发布  /carla/ego_vehicle/vehicle_control_cmd   (std_msgs/Float32MultiArray)
          数据格式 [throttle, steer, brake, reverse]

两种读取方式（自动选择，可用 --pygame / --terminal 强制指定）：
  * pygame   ：有图形界面时读取窗口键盘事件（与 standalone 模式一致）
  * terminal ：无图形环境下用 termios 原始模式直接读按键（虚拟机 / SSH 友好）

真实驾驶逻辑：S 在有速度时刹车减速；车速低于阈值时视为挂倒挡后退。
"""

import argparse
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

TOPIC = '/carla/ego_vehicle/vehicle_control_cmd'

# 终端模式按键映射（单字符）
TERMINAL_KEYS = {
    'w': 'fwd', 's': 'rev',
    'a': 'left', 'd': 'right',
    'q': 'left_fine', 'e': 'right_fine',
}


class KeyboardTeleopNode(Node):
    """读取键盘并发布控制指令。"""

    def __init__(self, use_pygame=None):
        super().__init__('keyboard_teleop_node')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('throttle_max', 0.6)
        self.declare_parameter('brake_max', 0.8)
        self.declare_parameter('steer_max', 0.6)
        self.declare_parameter('reverse_speed_threshold', 0.5)
        # 当前车速由仿真节点广播，这里订阅以判断是否挂倒挡
        self.declare_parameter('speed_topic', '/carla/ego_vehicle/speed')

        p = self.get_parameter
        self.throttle_max = float(p('throttle_max').value)
        self.brake_max = float(p('brake_max').value)
        self.steer_max = float(p('steer_max').value)
        self.rev_threshold = float(p('reverse_speed_threshold').value)
        rate = float(p('publish_rate').value)

        self.speed = 0.0
        self.keys = {k: False for k in
                     ('fwd', 'rev', 'left', 'right', 'left_fine', 'right_fine')}
        self._quit = False

        self.pub = self.create_publisher(Float32MultiArray, TOPIC, 10)
        self.create_subscription(
            Float32, p('speed_topic').value, self._on_speed, 10)

        # 选择键盘读取后端
        if use_pygame is None:
            use_pygame = _has_display()
        self.backend = 'pygame' if use_pygame else 'terminal'
        self.get_logger().info(f"键盘读取方式：{self.backend}")

        self.create_timer(1.0 / max(rate, 1.0), self._publish_cmd)
        self.get_logger().info(
            "键盘遥控就绪：W 前进 / S 刹车(静止时倒车) / A 左转 / D 右转 / ESC 退出")

    def _on_speed(self, msg):
        self.speed = float(msg.data)

    def _publish_cmd(self):
        """按真实驾驶逻辑合成控制量并发布。"""
        throttle = brake = 0.0
        reverse = False

        if self.keys['fwd']:
            throttle = self.throttle_max
        elif self.keys['rev']:
            if self.speed < self.rev_threshold:
                reverse, throttle = True, self.throttle_max * 0.8  # 挂倒挡
            else:
                brake = self.brake_max                             # 刹车减速

        steer = 0.0
        if self.keys['left']:
            steer -= self.steer_max
        if self.keys['right']:
            steer += self.steer_max
        if self.keys['left_fine']:
            steer -= self.steer_max * 0.25
        if self.keys['right_fine']:
            steer += self.steer_max * 0.25

        msg = Float32MultiArray()
        msg.data = [float(throttle), float(steer), float(brake), float(reverse)]
        self.pub.publish(msg)


# ------------------------------------------------------------------ 键盘后端
def _has_display():
    """判断是否存在可用图形显示（决定是否用 pygame 窗口读键）。"""
    import os
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


def _terminal_loop(node):
    """termios 原始模式读单键：无需图形界面，适合虚拟机 / SSH。"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        print("终端键盘遥控已启动：W/S/A/D 控制，ESC 或 Ctrl-C 退出")
        while rclpy.ok() and not node._quit:
            rclpy.spin_once(node, timeout_sec=0.0)
            if select.select([sys.stdin], [], [], 0.02)[0]:
                ch = sys.stdin.read(1)
                if ch == '\x1b':          # ESC 退出
                    node._quit = True
                    break
                low = ch.lower()
                if low in TERMINAL_KEYS:
                    name = TERMINAL_KEYS[low]
                    # 终端模式无「保持按下」语义，采用脉冲式点动
                    node.keys[name] = True
                    _pulse_off(node, name)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n终端键盘遥控已退出")


def _pulse_off(node, name, duration=0.15):
    """点动：短暂保持按键为按下，再释放（终端模式无 keyup 事件）。"""
    import threading
    import time

    def _release():
        time.sleep(duration)
        node.keys[name] = False
    threading.Thread(target=_release, daemon=True).start()


def _pygame_loop(node):
    """pygame 窗口读键：逐帧直读按键状态，与 standalone 模式一致。"""
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((480, 120))
    pygame.display.set_caption("CARLA 键盘遥控 W/S/A/D  ESC退出")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 22)
    try:
        while rclpy.ok() and not node._quit:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    node._quit = True
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    node._quit = True
            ks = pygame.key.get_pressed()
            node.keys.update({
                'fwd': bool(ks[pygame.K_w]),
                'rev': bool(ks[pygame.K_s]),
                'left': bool(ks[pygame.K_a]) or bool(ks[pygame.K_LEFT]),
                'right': bool(ks[pygame.K_d]) or bool(ks[pygame.K_RIGHT]),
                'left_fine': bool(ks[pygame.K_q]),
                'right_fine': bool(ks[pygame.K_e]),
            })
            screen.fill((20, 20, 20))
            txt = (f"v={node.speed:.1f} m/s  keys: "
                   + " ".join(f"{k[0]}{int(v)}" for k, v in node.keys.items()))
            screen.blit(font.render(txt, True, (0, 255, 0)), (10, 50))
            pygame.display.flip()
            rclpy.spin_once(node, timeout_sec=0.0)
            clock.tick(30)
    finally:
        pygame.quit()


def main(argv=None):
    parser = argparse.ArgumentParser(description="CARLA 键盘遥控节点")
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--pygame', action='store_true', help="强制使用 pygame 窗口读键")
    group.add_argument('--terminal', action='store_true', help="强制使用终端原始模式读键")
    args, _ = parser.parse_known_args(argv)

    rclpy.init(args=None)
    use_pygame = True if args.pygame else (False if args.terminal else None)
    node = KeyboardTeleopNode(use_pygame=use_pygame)
    try:
        if node.backend == 'pygame':
            _pygame_loop(node)
        else:
            _terminal_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
