"""
人形机器人键盘遥操作节点
========================
读取终端按键并发布 /cmd_vel 速度指令。

按键映射:
    W / S — 前进 / 后退 (vx)
    A / D — 左转 / 右转 (wz)
    空格   — 急停（所有速度归零）
    ESC    — 退出

说明：ros2 launch 启动的节点 stdin 指向 /dev/null（ROS 已知限制），
这里优先打开控制终端 /dev/tty 读键；没有控制终端时回退到 stdin。
"""

import os
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# 按键 -> (线速度符号, 角速度符号)
KEY_BINDINGS = {
    'w': (1.0, 0.0),   # 前进
    's': (-1.0, 0.0),  # 后退
    'a': (0.0, 1.0),   # 左转
    'd': (0.0, -1.0),  # 右转
    'KEY_UP': (1.0, 0.0),
    'KEY_DOWN': (-1.0, 0.0),
    'KEY_LEFT': (0.0, 1.0),
    'KEY_RIGHT': (0.0, -1.0),
}

HELP_TEXT = """
============================================================
  人形机器人键盘运动控制（发布 /cmd_vel）
------------------------------------------------------------
  W / S     前进 / 后退
  A / D     左转 / 右转
  空格       急停
  ESC        退出
  松开按键   停止
============================================================
"""


class KeyboardTeleopNode(Node):
    """键盘遥控节点"""

    def __init__(self):
        super().__init__('keyboard_teleop_node')

        # 参数
        self.declare_parameter('linear_scale', 0.3)    # 线速度 (m/s)
        self.declare_parameter('angular_scale', 0.4)   # 角速度 (rad/s)
        self.declare_parameter('publish_rate', 30.0)

        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.timer = self.create_timer(1.0 / pub_rate, self._publish_callback)

        # 终端设置（roslaunch 下优先 /dev/tty）
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

        self.get_logger().info('键盘遥控节点已启动 (W/S 前进后退, A/D 转向)')
        print(HELP_TEXT)

    def _get_key(self, timeout=0.03):
        """非阻塞读取按键（支持 ANSI 方向键转义序列）。"""
        if self.settings is None:
            return ''
        try:
            tty.setraw(self._fd)
            rlist, _, _ = select.select([self._fd], [], [], timeout)
            key = ''
            if rlist:
                c = os.read(self._fd, 1).decode('utf-8', errors='ignore')
                if c == '\x1b':
                    # ESC：可能是方向键转义序列的开头，也可能是独立 ESC 键
                    r2, _, _ = select.select([self._fd], [], [], 0.03)
                    if r2:
                        c2 = os.read(self._fd, 1).decode('utf-8', errors='ignore')
                        if c2 == '[':
                            r3, _, _ = select.select([self._fd], [], [], 0.03)
                            if r3:
                                c3 = os.read(self._fd, 1).decode('utf-8', errors='ignore')
                                if c3 == 'A':
                                    key = 'KEY_UP'
                                elif c3 == 'B':
                                    key = 'KEY_DOWN'
                                elif c3 == 'C':
                                    key = 'KEY_RIGHT'
                                elif c3 == 'D':
                                    key = 'KEY_LEFT'
                            else:
                                key = '\x1b'   # ESC + '[' 无后续字符，视为 ESC
                        else:
                            key = '\x1b'       # ESC + 其他字符，视为 ESC
                    else:
                        key = '\x1b'           # 独立 ESC 键
                else:
                    key = c
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self.settings)
            return key
        except Exception:
            return ''

    def _publish_callback(self):
        """定时读取按键并发布速度指令（松开按键即停止）。"""
        key = self._get_key()
        if key == '\x1b':
            # ESC：发布零速度后退出（终端设置由 destroy_node 恢复）
            self.twist = Twist()
            self.pub_cmd_vel.publish(self.twist)
            raise KeyboardInterrupt
        elif key in (' ', '\x03'):
            # 急停
            self.twist = Twist()
            if key == '\x03':
                raise KeyboardInterrupt
        elif key in KEY_BINDINGS:
            vx_s, wz_s = KEY_BINDINGS[key]
            self.twist = Twist()
            self.twist.linear.x = vx_s * self.linear_scale
            self.twist.angular.z = wz_s * self.angular_scale
        else:
            # 松开 / 未知键：停止
            self.twist = Twist()
        self.pub_cmd_vel.publish(self.twist)

    def publish_zero_velocity(self):
        """发布零速度（退出前调用，避免仿真继续执行最后一条非零指令）。"""
        try:
            self.pub_cmd_vel.publish(Twist())
        except Exception:
            pass

    def destroy_node(self):
        """恢复终端设置。"""
        if self.settings is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self.settings)
            except Exception:
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
        # 退出前发布零速度，避免遥控节点停止后仿真继续执行最后一条非零指令
        node.publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
