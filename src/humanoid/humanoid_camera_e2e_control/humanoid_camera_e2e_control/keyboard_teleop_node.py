# -*- coding: utf-8 -*-
"""
keyboard_teleop_node.py —— 键盘遥操作节点（采集模式使用）。

读取终端按键并发布 /cmd_vel，配合 mujoco_sim_node 完成人在环驾驶，
供 data_collector（standalone 采集，见 scripts/collect_data.py）在 ROS2 下复用同一
套驾驶交互。

按键：W/S 前进后退，A/D 左转右转，空格急停，ESC 退出。
"""

import os
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


KEY_BINDINGS = {
    'w': (1.0, 0.0), 's': (-1.0, 0.0),
    'a': (0.0, 1.0), 'd': (0.0, -1.0),
    'KEY_UP': (1.0, 0.0), 'KEY_DOWN': (-1.0, 0.0),
    'KEY_LEFT': (0.0, 1.0), 'KEY_RIGHT': (0.0, -1.0),
}

HELP_TEXT = """
============================================================
  人形机器人键盘驾驶（采集模式，发布 /cmd_vel）
------------------------------------------------------------
  W / S  前进 / 后退      A / D  左转 / 右转
  空格    急停             ESC    退出
============================================================
"""


class KeyboardTeleopNode(Node):
    """键盘遥控节点。"""

    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.declare_parameter('linear_scale', 0.22)
        self.declare_parameter('angular_scale', 0.35)
        self.declare_parameter('publish_rate', 30.0)

        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.timer = self.create_timer(1.0 / pub_rate, self._publish)

        self._close_fd = False
        try:
            self._fd = os.open('/dev/tty', os.O_RDONLY)
            self._close_fd = True
        except OSError:
            self._fd = sys.stdin.fileno()
        try:
            self.settings = termios.tcgetattr(self._fd)
        except Exception:
            self.settings = None

        self.get_logger().info('键盘遥控节点已启动')
        print(HELP_TEXT)

    def _get_key(self, timeout=0.03):
        if self.settings is None:
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
                                       'C': 'KEY_RIGHT', 'D': 'KEY_LEFT'}.get(c3, '\x1b')
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
        except Exception:
            return ''

    def _publish(self):
        key = self._get_key()
        if key == '\x1b':
            self.twist = Twist()
            self.pub_cmd.publish(self.twist)
            raise KeyboardInterrupt
        if key in (' ', '\x03'):
            self.twist = Twist()
            if key == '\x03':
                raise KeyboardInterrupt
        elif key in KEY_BINDINGS:
            vx, wz = KEY_BINDINGS[key]
            self.twist = Twist()
            self.twist.linear.x = vx * self.linear_scale
            self.twist.angular.z = wz * self.angular_scale
        else:
            self.twist = Twist()
        self.pub_cmd.publish(self.twist)

    def destroy_node(self):
        if self.settings is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self.settings)
            except Exception:                                         # noqa: BLE001
                pass
        if self._close_fd:
            try:
                os.close(self._fd)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
