"""Keyboard teleoperation for the MuJoCo auto-parking car."""

import select
import sys
import termios
import tty
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


QUIT_KEYS = {'q', 'Q', 'ESC', '\x03'}
STOP_KEYS = {' ', '0'}

HELP_TEXT = """
自动泊车小车键盘控制：
  W / ↑   前进       S / ↓   后退
  A / ←   左转       D / →   右转
  空格    急停       Q / Esc 退出
说明：按键后指令会保持，直到按另一个方向键或空格。
"""


def command_for_key(
    key: str,
    linear_speed: float,
    angular_speed: float,
) -> Optional[Tuple[float, float]]:
    """Map one terminal key to a (linear.x, angular.z) command."""
    mapping = {
        'w': (linear_speed, 0.0),
        'W': (linear_speed, 0.0),
        'KEY_UP': (linear_speed, 0.0),
        's': (-linear_speed, 0.0),
        'S': (-linear_speed, 0.0),
        'KEY_DOWN': (-linear_speed, 0.0),
        'a': (0.0, angular_speed),
        'A': (0.0, angular_speed),
        'KEY_LEFT': (0.0, angular_speed),
        'd': (0.0, -angular_speed),
        'D': (0.0, -angular_speed),
        'KEY_RIGHT': (0.0, -angular_speed),
    }
    if key in STOP_KEYS:
        return 0.0, 0.0
    return mapping.get(key)


def is_quit_key(key: str) -> bool:
    """Return True if the key asks the teleop node to stop."""
    return key in QUIT_KEYS


class KeyboardTeleopNode(Node):
    """Publish /cmd_vel from non-blocking terminal key presses."""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop_node')
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.9)
        self.declare_parameter('publish_rate', 20.0)

        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')

        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._command = (0.0, 0.0)
        self._stopped = False
        self._terminal_settings = None
        try:
            self._terminal_settings = termios.tcgetattr(sys.stdin)
        except Exception:
            self.get_logger().warning(
                '当前终端不可用，键盘节点只会发布零速度 / '
                'stdin is not a TTY; publishing zero velocity'
            )
        self.create_timer(1.0 / publish_rate, self._on_timer)
        self.get_logger().info('键盘控制已启动 / keyboard teleop started')
        print(HELP_TEXT)

    def _read_key(self, timeout: float = 0.05) -> str:
        if self._terminal_settings is None:
            return ''
        try:
            tty.setraw(sys.stdin.fileno())
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            key = ''
            if ready:
                char = sys.stdin.read(1)
                if char == '\x1b':
                    ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if ready:
                        second = sys.stdin.read(1)
                        if second == '[':
                            ready, _, _ = select.select(
                                [sys.stdin], [], [], 0.05
                            )
                            if ready:
                                third = sys.stdin.read(1)
                                key = {
                                    'A': 'KEY_UP',
                                    'B': 'KEY_DOWN',
                                    'C': 'KEY_RIGHT',
                                    'D': 'KEY_LEFT',
                                }.get(third, 'ESC')
                        else:
                            key = 'ESC'
                else:
                    key = char
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, self._terminal_settings
            )
            return key
        except Exception:
            return ''

    def _on_timer(self) -> None:
        key = self._read_key()
        if is_quit_key(key):
            self._stopped = True
            self._command = (0.0, 0.0)
            self.get_logger().info('收到退出键，速度已归零 / stop requested')
        elif key:
            command = command_for_key(
                key, self._linear_speed, self._angular_speed
            )
            if command is not None:
                self._command = command

        message = Twist()
        message.linear.x = self._command[0]
        message.angular.z = self._command[1]
        self._publisher.publish(message)

    def destroy_node(self) -> None:
        if self._terminal_settings is not None:
            try:
                termios.tcsetattr(
                    sys.stdin, termios.TCSADRAIN, self._terminal_settings
                )
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
