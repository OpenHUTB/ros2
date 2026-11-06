"""
keyboard_node.py —— 键盘遥操作节点。
映射：w/s 前后（线速度），a/d 左右转（角速度），q/e 上浮/下潜，空格急停。
为什么用 termios 而不是 pygame：无显示虚拟机/SSH 下 pygame 会失败，
termios 在终端直接读 raw 字符，依赖最少。
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP_MSG = """
  ROV 键盘控制：
    w/s   前进/后退
    a/d   左转/右转
    q/e   上浮/下潜
    空格  急停
    Ctrl+C 退出
"""


class KeyboardNode(Node):
    def __init__(self):
        super().__init__("rov_keyboard_node")
        self.cmd_topic = self.declare_parameter("cmd_topic", "/rov_teleop/cmd_vel").value
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_timer(0.1, self._loop)
        self._settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info(HELP_MSG)

    def _read_key(self) -> str:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        return ch

    def _loop(self):
        try:
            ch = self._read_key()
        except Exception:
            return
        twist = Twist()
        if ch == "w":
            twist.linear.x = 0.5
        elif ch == "s":
            twist.linear.x = -0.5
        elif ch == "a":
            twist.angular.z = 0.5
        elif ch == "d":
            twist.angular.z = -0.5
        elif ch == "q":
            twist.linear.z = 0.3
        elif ch == "e":
            twist.linear.z = -0.3
        elif ch == " ":
            pass  # 全 0
        elif ch in ("\x03",):
            raise KeyboardInterrupt
        else:
            return
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node._settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
