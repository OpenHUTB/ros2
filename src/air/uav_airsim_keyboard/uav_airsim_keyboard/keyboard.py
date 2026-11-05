#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘指令发布节点：读终端按键 -> 发布 uav/cmd_vel，并可调用起飞/降落服务。

按键（按住生效，松开约 0.25 s 后自动停车）：
    w / s   前进 / 后退
    a / d   左移 / 右移
    r / f   上升 / 下降
    q / e   左偏航 / 右偏航
    t       起飞（调用 uav/takeoff 服务）
    g       降落（调用 uav/land 服务）
    Ctrl+C  退出（会触发桥接节点降落）

两种启动方式：
    ros2 run uav_airsim_keyboard keyboard     # 需要真实终端（isatty 为真）
    ros2 run uav_airsim_keyboard main         # 桥接 + 键盘，一条命令跑完整个模块

注意：`ros2 launch` 不会把 stdin 转发给子进程，因此 launch 起来时键盘不可用，
      这是 ROS 2 的已知行为，不是本模块的 bug；需要键盘时请用上面两条命令之一。
"""

import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger

# 按键 -> (linear.x, linear.y, linear.z, angular.z) 的方向系数
KEY_DIRECTION = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, -1.0, 0.0, 0.0),
    'd': (0.0, 1.0, 0.0, 0.0),
    'r': (0.0, 0.0, 1.0, 0.0),
    'f': (0.0, 0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 0.0, 1.0),
    'e': (0.0, 0.0, 0.0, -1.0),
}


class KeyboardPublisher(Node):
    """把终端按键转成 geometry_msgs/Twist。"""

    def __init__(self):
        super().__init__('uav_keyboard')
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('yaw_rate', 45.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('key_hold_timeout', 0.25)

        self._speed = float(self.get_parameter('speed').value)
        self._yaw_rate = float(self.get_parameter('yaw_rate').value)
        rate = float(self.get_parameter('publish_rate').value)
        self._hold = float(self.get_parameter('key_hold_timeout').value)

        self._pressed = {}
        self._pressed_lock = threading.Lock()
        self._stop = threading.Event()

        self._pub = self.create_publisher(Twist, 'uav/cmd_vel', 10)
        self._takeoff_cli = self.create_client(Trigger, 'uav/takeoff')
        self._land_cli = self.create_client(Trigger, 'uav/land')

        self._thread = threading.Thread(
            target=self._read_loop, name='keyboard-reader', daemon=True)
        self._thread.start()
        self._timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            '键盘遥控已启动：w/s 前后, a/d 左右, r/f 升降, q/e 偏航, t 起飞, g 降落')

    # ---------------------------------------------------------------- 按键读
    def _read_loop(self):
        fd = sys.stdin.fileno()
        try:
            old_attr = termios.tcgetattr(fd)
        except termios.error as exc:
            self.get_logger().error('无法配置终端（不是 TTY 吗？）: %s' % exc)
            return
        try:
            tty.setcbreak(fd)
            while not self._stop.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                char = sys.stdin.read(1)
                if not char:
                    break
                self._handle_key(char)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)

    def _handle_key(self, char):
        char = char.lower()
        if char == 't':
            self._call_service(self._takeoff_cli, '起飞')
        elif char == 'g':
            self._call_service(self._land_cli, '降落')
        elif char in KEY_DIRECTION:
            with self._pressed_lock:
                self._pressed[char] = time.monotonic()

    def _call_service(self, client, name):
        if not client.service_is_ready():
            self.get_logger().warning('%s 服务尚未就绪，请稍后重试' % name)
            return
        future = client.call_async(Trigger.Request())

        def _done(fut):
            try:
                res = fut.result()
                self.get_logger().info('%s: %s' % (name, res.message))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('%s 调用失败: %s' % (name, exc))

        future.add_done_callback(_done)

    # ----------------------------------------------------------------- 发布
    def _publish(self):
        now = time.monotonic()
        with self._pressed_lock:
            active = [k for k, t in self._pressed.items() if now - t <= self._hold]
            # 清掉过期按键，避免字典无限增长
            self._pressed = {k: t for k, t in self._pressed.items() if now - t <= self._hold}

        msg = Twist()
        for key in active:
            dx, dy, dz, dyaw = KEY_DIRECTION[key]
            msg.linear.x += dx * self._speed
            msg.linear.y += dy * self._speed
            msg.linear.z += dz * self._speed
            msg.angular.z += dyaw * self._yaw_rate
        self._pub.publish(msg)

    def stop(self):
        self._stop.set()


def main(argv=None):
    rclpy.init(args=sys.argv if argv is None else argv)
    node = KeyboardPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
