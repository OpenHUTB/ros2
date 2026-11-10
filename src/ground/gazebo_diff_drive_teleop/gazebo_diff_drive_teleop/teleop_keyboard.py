#!/usr/bin/env python3
"""ROS 2 差速小车键盘遥控节点。

读取终端按键，并以固定频率向 /cmd_vel 发布 geometry_msgs/Twist 消息。
差速小车只需要线速度 linear.x 和角速度 angular.z：
  - 前进/后退：linear.x 为正/负
  - 左转/右转：angular.z 为正/负（原地转向）

按键：
  w / ↑    前进
  x / ↓    后退
  a / ←    左转
  d / →    右转
  s / 空格  停止
  + / -    增减线速度
  q / Ctrl-C 退出（退出前自动停车）
"""

import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# 方向键在终端中发送的 ANSI 转义序列
ARROW_KEYS = {
    '\x1b[A': 'up',
    '\x1b[B': 'down',
    '\x1b[C': 'right',
    '\x1b[D': 'left',
}


class TeleopKeyboard(Node):
    """读取键盘输入并持续发布差速小车速度指令。"""

    def __init__(self):
        super().__init__('teleop_keyboard')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('linear_speed', 0.3)   # m/s
        self.declare_parameter('angular_speed', 1.0)  # rad/s
        self.declare_parameter('publish_rate', 20.0)   # Hz

        topic = str(self.get_parameter('cmd_vel_topic').value)
        self.linear_speed = max(0.0, float(self.get_parameter('linear_speed').value))
        self.angular_speed = max(0.0, float(self.get_parameter('angular_speed').value))
        self.publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(Twist, topic, qos)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self._key_buffer = b''
        self._stdin_fd = sys.stdin.fileno()
        self._original_term_settings = None
        self._running = True

        if not sys.stdin.isatty():
            raise RuntimeError(
                '标准输入不是交互式终端，无法读取按键。'
                '请在终端中直接运行本节点。'
            )

    # ------------------------------------------------------------------
    # 终端设置
    # ------------------------------------------------------------------
    def _enable_cbreak(self):
        """关闭行缓冲和回显，使按键可以立即被读取。"""
        self._original_term_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

    def _restore_terminal(self):
        if self._original_term_settings is not None:
            termios.tcsetattr(
                self._stdin_fd, termios.TCSADRAIN, self._original_term_settings
            )
            self._original_term_settings = None

    # ------------------------------------------------------------------
    # 键盘输入
    # ------------------------------------------------------------------
    def _parse_keys(self, data: bytes):
        """把一次读到的字节流拆成按键，并识别方向键转义序列。"""
        self._key_buffer += data
        keys = []

        while self._key_buffer:
            if self._key_buffer.startswith(b'\x1b'):
                # 方向键至少 3 个字节，数据不完整时等待下一次读取
                if len(self._key_buffer) < 3:
                    break
                seq = self._key_buffer[:3].decode('ascii', errors='ignore')
                self._key_buffer = self._key_buffer[3:]
                keys.append(ARROW_KEYS.get(seq, seq))
            else:
                key = self._key_buffer[:1].decode('utf-8', errors='ignore')
                self._key_buffer = self._key_buffer[1:]
                keys.append(ARROW_KEYS.get(key, key))

        return keys

    def _read_keys(self, timeout: float):
        """在 timeout 秒内读取所有可用按键，无输入时返回空列表。"""
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return []

        data = os.read(self._stdin_fd, 64)
        if not data:
            return ['q']  # 标准输入被关闭，按退出处理
        return self._parse_keys(data)

    def _handle_key(self, key: str):
        """根据按键更新目标速度；按需要返回 False 表示退出。"""
        if key in ('w', 'W', 'up'):
            self.linear_x = self.linear_speed
            self.angular_z = 0.0
        elif key in ('x', 'X', 'down'):
            self.linear_x = -self.linear_speed
            self.angular_z = 0.0
        elif key in ('a', 'A', 'left'):
            self.linear_x = 0.0
            self.angular_z = self.angular_speed
        elif key in ('d', 'D', 'right'):
            self.linear_x = 0.0
            self.angular_z = -self.angular_speed
        elif key in ('s', 'S', ' ', 'k', 'K'):
            self.linear_x = 0.0
            self.angular_z = 0.0
        elif key in ('+', '='):
            self.linear_speed = min(2.0, self.linear_speed + 0.1)
            self._apply_speed_change()
            self.get_logger().info(
                f'线速度: {self.linear_speed:.2f} m/s, '
                f'角速度: {self.angular_speed:.2f} rad/s'
            )
        elif key in ('-', '_'):
            self.linear_speed = max(0.0, self.linear_speed - 0.1)
            self._apply_speed_change()
            self.get_logger().info(
                f'线速度: {self.linear_speed:.2f} m/s, '
                f'角速度: {self.angular_speed:.2f} rad/s'
            )
        elif key in ('q', 'Q', '\x03'):
            return False
        return True

    def _apply_speed_change(self):
        """调速后让当前运动立即使用新速度。"""
        if self.linear_x > 0.0:
            self.linear_x = self.linear_speed
        elif self.linear_x < 0.0:
            self.linear_x = -self.linear_speed

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher.publish(msg)

    def stop_robot(self):
        """向控制器发送若干次零速度，确保退出时小车停下。"""
        msg = Twist()
        for _ in range(3):
            try:
                self.publisher.publish(msg)
                # 给 DDS 一点时间把最后几条消息发出去
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception:  # 上下文已被关闭时也要尽量完成停车
                break

    def print_usage(self):
        self.get_logger().info('差速小车键盘遥控已启动')
        self.get_logger().info('  w / ↑ : 前进      x / ↓ : 后退')
        self.get_logger().info('  a / ← : 左转      d / → : 右转')
        self.get_logger().info('  s / 空格 : 停止   q / Ctrl-C : 退出')
        self.get_logger().info('  + / - : 调整线速度')
        self.get_logger().info(
            f'当前线速度 {self.linear_speed:.2f} m/s，'
            f'角速度 {self.angular_speed:.2f} rad/s'
        )

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def run(self):
        self._enable_cbreak()
        self.print_usage()
        period = 1.0 / self.publish_rate

        try:
            while rclpy.ok() and self._running:
                for key in self._read_keys(period):
                    if not self._handle_key(key):
                        self._running = False
                        break
                self._publish_cmd()
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            self._restore_terminal()
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopKeyboard()
        node.run()
    except RuntimeError as exc:
        print(f'[teleop_keyboard] 错误: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
