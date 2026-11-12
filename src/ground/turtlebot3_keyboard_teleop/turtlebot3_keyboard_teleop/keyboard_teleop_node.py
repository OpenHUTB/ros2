#!/usr/bin/env python3
"""键盘 teleop ROS2 节点：timer 周期发布，超时自动归零。"""
import atexit
import math
import signal
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .keyboard_logic import KeyLatch, Limits


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("turtlebot3_keyboard_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        max_vx = self.declare_parameter("max_vx", 0.5).value
        max_wz = self.declare_parameter("max_wz", 1.0).value
        timeout_sec = self.declare_parameter("timeout_sec", 0.5).value
        # 检查 NaN/Inf 和非法值
        if not isinstance(max_vx, (int, float)) or math.isnan(max_vx) or math.isinf(max_vx) or max_vx <= 0:
            self.get_logger().warn(f"max_vx={max_vx} 非法，重置为 0.5")
            max_vx = 0.5
        if not isinstance(max_wz, (int, float)) or math.isnan(max_wz) or math.isinf(max_wz) or max_wz <= 0:
            self.get_logger().warn(f"max_wz={max_wz} 非法，重置为 1.0")
            max_wz = 1.0
        if not isinstance(timeout_sec, (int, float)) or math.isnan(timeout_sec) or math.isinf(timeout_sec) or timeout_sec <= 0:
            self.get_logger().warn(f"timeout_sec={timeout_sec} 非法，重置为 0.5")
            timeout_sec = 0.5
        self.limits = Limits(max_vx=max_vx, max_wz=max_wz, timeout_sec=timeout_sec)
        self.latch = KeyLatch(limits=self.limits)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._fd = None
        self._old_term = None
        self.timer = self.create_timer(0.05, self._on_timer)
        self.get_logger().info("w/s/a/d 移动，空格停车，q 或 Ctrl-C 退出")

    def _on_timer(self):
        with self._lock:
            vx, wz = self.latch.get_cmd()
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

    def start_keyboard_listener(self):
        if not sys.stdin.isatty():
            self.get_logger().warn("stdin 不是终端，键盘遥控不可用，节点只发零速度")
            self._stop.wait()
            return
        try:
            import termios
            import tty
        except ImportError:
            self.get_logger().warn("当前平台不支持终端原始模式读键（缺少 termios）")
            self._stop.wait()
            return
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        self._fd = fd
        self._old_term = old
        atexit.register(self._restore_terminal)
        try:
            tty.setraw(fd)
            while not self._stop.is_set():
                ch = sys.stdin.read(1)
                if not ch:
                    break
                if ch in ("q", "\x03"):
                    self._stop.set()
                    break
                if ch.lower() in ("w", "s", "a", "d", " "):
                    with self._lock:
                        self.latch.press(ch)
        finally:
            self._restore_terminal()

    def _restore_terminal(self):
        if self._fd is not None and self._old_term is not None:
            import termios
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            except Exception:
                pass
            self._fd = None
            self._old_term = None

    def stop(self):
        self._stop.set()


def main():
    rclpy.init()
    node = KeyboardTeleopNode()
    # signal.signal 必须在主线程调用，不能放后台监听线程里
    signal.signal(signal.SIGTERM, lambda *_: node.stop())
    signal.signal(signal.SIGINT, lambda *_: node.stop())
    t = threading.Thread(target=node.start_keyboard_listener, daemon=True)
    t.start()
    try:
        while not node._stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        # 退出前连发几次零速，确保车辆停住
        for _ in range(5):
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            node.pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
