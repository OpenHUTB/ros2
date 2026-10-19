"""
扰动遥控节点
============
读取终端按键，向 ``/balance/push`` 发布扰动力（geometry_msgs/Vector3），
并可向 ``/balance/reset`` 发送复位指令，用于交互式检验平衡恢复能力。

按键映射::

    W / S      向前 / 向后推 (沿 ∓x)
    A / D      向左 / 向右推 (沿 ±y)
    Q / E      左前 / 右后斜向推 (45°)
    + / -      提高 / 降低推扰力度（步进 20 N）
    空格       使机器人复位
    ESC        退出

说明：ros2 launch 启动的节点 stdin 指向 /dev/null（ROS 已知限制），
这里优先打开控制终端 /dev/tty 读键；没有控制终端时回退到 stdin。
"""

import os
import sys
try:
    import termios
    import tty
    import select
except ImportError:
    termios = None
    tty = None
    select = None
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty

from .keymap import direction_label, key_to_direction

HELP_TEXT = """
============================================================
  人形机器人扰动遥控（发布 /balance/push）
------------------------------------------------------------
  W / S      向前 / 向后推
  A / D      向左 / 向右推
  Q / E      左前 / 右后斜向推
  ↑ / ↓      同 W / S（方向键）
  ← / →      同 A / D（方向键）
  + / -      推扰力度 +20 / -20 N
  空格       复位
  ESC        退出
============================================================
"""


class PushTeleopNode(Node):
    """扰动遥控节点。"""

    def __init__(self):
        super().__init__('push_teleop_node')

        self.declare_parameter('force', 120.0)        # 当前推扰力度 (N)
        self.declare_parameter('force_step', 20.0)    # 力度步进 (N)
        self.declare_parameter('force_min', 20.0)
        self.declare_parameter('force_max', 400.0)

        self.force = self.get_parameter('force').get_parameter_value().double_value
        self.step = self.get_parameter('force_step').get_parameter_value().double_value
        self.fmin = self.get_parameter('force_min').get_parameter_value().double_value
        self.fmax = self.get_parameter('force_max').get_parameter_value().double_value

        self.pub_push = self.create_publisher(Vector3, '/balance/push', 10)
        self.pub_reset = self.create_publisher(Empty, '/balance/reset', 10)
        self.timer = self.create_timer(0.03, self._poll_callback)

        # 终端设置（launch 下优先 /dev/tty）
        self._close_fd = False
        self.settings = None
        self._fd = None
        if termios is not None:
            try:
                self._fd = os.open('/dev/tty', os.O_RDONLY)
                self._close_fd = True
            except OSError:
                self._fd = sys.stdin.fileno()
            try:
                self.settings = termios.tcgetattr(self._fd)
            except Exception:                                     # noqa: BLE001
                self.settings = None

        self.get_logger().info(f'扰动遥控节点已启动（当前力度 {self.force:.0f} N）')
        print(HELP_TEXT)

    # ------------------------------------------------------------------
    def _get_key(self, timeout=0.02):
        """非阻塞读取按键（支持 ANSI 方向键转义序列）。"""
        if self.settings is None or tty is None or select is None or termios is None:
            return ''
        try:
            tty.setraw(self._fd)
            rlist, _, _ = select.select([self._fd], [], [], timeout)
            key = ''
            if rlist:
                c = os.read(self._fd, 1).decode('utf-8', errors='ignore')
                if c == '\x1b':
                    r2, _, _ = select.select([self._fd], [], [], 0.03)
                    if r2:
                        c2 = os.read(self._fd, 1).decode('utf-8', errors='ignore')
                        if c2 == '[':
                            r3, _, _ = select.select([self._fd], [], [], 0.03)
                            if r3:
                                c3 = os.read(self._fd, 1).decode('utf-8', errors='ignore')
                                key = {'A': 'KEY_UP', 'B': 'KEY_DOWN',
                                       'C': 'KEY_RIGHT', 'D': 'KEY_LEFT'}.get(c3, '')
                            else:
                                key = '\x1b'
                        else:
                            key = '\x1b'
                    else:
                        key = '\x1b'
                else:
                    key = c
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self.settings)
            return key
        except Exception:                                     # noqa: BLE001
            return ''

    def _poll_callback(self):
        """定时读取按键并发布扰动 / 复位指令。"""
        key = self._get_key()
        if not key:
            return
        if key == '\x1b':
            raise KeyboardInterrupt
        if key == ' ':
            self.pub_reset.publish(Empty())
            self.get_logger().info('已发送复位指令')
            return
        if key in ('+', '='):
            self.force = min(self.fmax, self.force + self.step)
            self.get_logger().info(f'推扰力度 → {self.force:.0f} N')
            return
        if key in ('-', '_'):
            self.force = max(self.fmin, self.force - self.step)
            self.get_logger().info(f'推扰力度 → {self.force:.0f} N')
            return
        # 方向键键名为大写 KEY_*，字母键需兼容大小写；统一交给 keymap 归一化，
        # 避免在此处再写一次 lower() 造成两者不一致（方向键会被静默忽略）。
        deg = key_to_direction(key)
        if deg is not None:
            rad = math.radians(deg)
            msg = Vector3(x=self.force * math.cos(rad), y=self.force * math.sin(rad), z=0.0)
            self.pub_push.publish(msg)
            self.get_logger().info(
                f'施加扰动 {self.force:.0f} N @ {deg:.0f}°（{direction_label(deg)}）')

    # ------------------------------------------------------------------
    def destroy_node(self):
        """恢复终端设置。"""
        if self.settings is not None and termios is not None and self._fd is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self.settings)
            except Exception:                                 # noqa: BLE001
                pass
        if self._close_fd:
            try:
                os.close(self._fd)
            except Exception:                                 # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PushTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
