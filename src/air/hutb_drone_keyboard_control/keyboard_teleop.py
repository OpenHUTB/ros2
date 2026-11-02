#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
keyboard_teleop.py - 键盘遥控节点

在终端中读取键盘按键，将控制指令以 geometry_msgs/Twist 发布到 /cmd_vel 话题。

按键说明（小写）：
    w / s : 前进 / 后退        （linear.x  增 / 减）
    a / d : 左移 / 右移        （linear.y  增 / 减）
    SPACE : 上升               （linear.z  增）
    c     : 下降               （linear.z  减）
    q / e : 左旋 / 右旋        （angular.z 增 / 减）
    g / b : 增大 / 减小速度步长
    k     : 急停（所有速度清零）
    h     : 重新打印帮助

说明：0.4 秒内没有检测到任何按键输入时，速度会按比例衰减到 0，
     即"松手即减速"，避免无人机失漂；持续按住按键可保持匀速运动
     （终端按住会连续上报键值）。
"""

import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP_TEXT = """
====================== 键盘控制说明 ======================
  w / s : 前进 / 后退          a / d : 左移 / 右移
  SPACE : 上升                 c     : 下降
  q / e : 左旋 / 右旋          g / b : 增大 / 减小速度步长
  k     : 急停（清零所有速度）  h     : 打印本帮助
  提示: 松开按键 0.4 秒后速度自动衰减为 0（缓降悬停）
==========================================================
"""

# 0.4 s 无按键输入则开始速度衰减
IDLE_TIMEOUT = 0.4
# 每个发布周期(0.1s)速度衰减系数
DECAY_FACTOR = 0.75


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        # 可调参数
        self.declare_parameter('speed_step', 1.0)      # 水平速度步长 (m/s)
        self.declare_parameter('vertical_step', 0.8)   # 垂直速度步长 (m/s)
        self.declare_parameter('yaw_step', 30.0)       # 偏航角速度步长 (deg/s)
        self.declare_parameter('max_linear', 8.0)      # 水平速度上限 (m/s)
        self.declare_parameter('max_vertical', 4.0)    # 垂直速度上限 (m/s)
        self.declare_parameter('max_yaw_rate', 120.0)  # 偏航角速度上限 (deg/s)
        self.declare_parameter('publish_rate', 10.0)   # 指令发布频率 (Hz)

        self._step = float(self.get_parameter('speed_step').value)
        self._vstep = float(self.get_parameter('vertical_step').value)
        self._yaw_step = float(self.get_parameter('yaw_step').value)
        self._max_lin = float(self.get_parameter('max_linear').value)
        self._max_ver = float(self.get_parameter('max_vertical').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rate').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 当前速度指令（加锁保护，键盘线程写、发布定时器读）
        self._twist = Twist()
        self._lock = threading.Lock()
        self._last_key_time = time.time()

        # 键盘读取线程（termios 原始模式，仅支持 Linux）
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

        # 定时发布
        self.create_timer(1.0 / rate, self._publish_loop)

        self.get_logger().info('键盘遥控节点已启动，话题: /cmd_vel')
        print(HELP_TEXT)

    # ---------------- 键盘处理 ----------------

    def _key_loop(self):
        """后台线程: 终端原始模式读键, 调整目标速度。仅 Linux 可用。"""
        if not sys.stdin.isatty():
            self.get_logger().error('stdin 不是终端, 键盘控制不可用')
            return
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch:
                    self._handle_key(ch)
                    self._last_key_time = time.time()
                time.sleep(0.01)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'键盘线程异常退出: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    def _handle_key(self, ch: str):
        with self._lock:
            t = self._twist
            if ch == 'w':
                t.linear.x += self._step
            elif ch == 's':
                t.linear.x -= self._step
            elif ch == 'a':
                t.linear.y += self._step
            elif ch == 'd':
                t.linear.y -= self._step
            elif ch == ' ':          # SPACE
                t.linear.z += self._vstep
            elif ch == 'c':
                t.linear.z -= self._vstep
            elif ch == 'q':
                t.angular.z += self._yaw_step
            elif ch == 'e':
                t.angular.z -= self._yaw_step
            elif ch == 'g':
                self._step = min(self._step + 0.5, self._max_lin)
                print(f'\n[速度步长] {self._step:.1f} m/s')
            elif ch == 'b':
                self._step = max(self._step - 0.5, 0.5)
                print(f'\n[速度步长] {self._step:.1f} m/s')
            elif ch == 'k':
                self._reset_twist(t)
                print('\n[急停] 速度已清零, 悬停')
            elif ch == 'h':
                print(HELP_TEXT)
            else:
                return  # 未定义按键, 不刷新时间戳

    @staticmethod
    def _reset_twist(t: Twist):
        t.linear.x = 0.0
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.z = 0.0

    # ---------------- 指令发布 ----------------

    def _publish_loop(self):
        with self._lock:
            t = self._twist
            # 超时未按键 → 速度衰减（松手即缓停）
            if time.time() - self._last_key_time > IDLE_TIMEOUT:
                t.linear.x *= DECAY_FACTOR
                t.linear.y *= DECAY_FACTOR
                t.linear.z *= DECAY_FACTOR
                t.angular.z *= DECAY_FACTOR
                if all(abs(v) < 0.05 for v in
                       (t.linear.x, t.linear.y, t.linear.z, t.angular.z)):
                    self._reset_twist(t)

            # 限幅
            t.linear.x = max(-self._max_lin, min(self._max_lin, t.linear.x))
            t.linear.y = max(-self._max_lin, min(self._max_lin, t.linear.y))
            t.linear.z = max(-self._max_ver, min(self._max_ver, t.linear.z))
            t.angular.z = max(-self._max_yaw, min(self._max_yaw, t.angular.z))

            self.pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前清零速度，保证无人机悬停
        msg = Twist()
        node.pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
