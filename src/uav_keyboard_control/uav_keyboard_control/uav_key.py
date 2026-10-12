import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


msg = """
=====================
无人机/小车 键盘控制
W : 向前    S : 向后
A : 向左    D : 向右
Q : 左转    E : 右转
空格 : 停止所有运动
CTRL+C 退出程序
=====================
"""

#按键映射
move_bindings = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0, 0.0),
    'd': (0.0, -1.0, 0.0, 0.0),
    'q': (0.0, 0.0, 0.0, 1.0),
    'e': (0.0, 0.0, 0.0, -1.0),
    ' ': (0.0, 0.0, 0.0, 0.0),
}

def get_key():
    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            key = sys.stdin.read(1)
        else:
            key = ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
    return key


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__("uav_keyboard_node")
        # 发布话题 /cmd_vel
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.twist = Twist()

    def send_twist(self, x, y, z, angular_z):
        self.twist.linear.x = x
        self.twist.linear.y = y
        self.twist.angular.z = angular_z
        self.pub.publish(self.twist)


def main():
    rclpy.init()
    node = KeyboardControlNode()
    print(msg)
    try:
        while True:
            k = get_key()
            if k in move_bindings:
                vx, vy, vz, az = move_bindings[k]
                node.send_twist(vx, vy, vz, az)
            # Ctrl + C
            elif k == "\x03":
                break
    except Exception as err:
        print(f"异常：{err}")
    finally:
        #退出，发布停止指令
        stop_msg = Twist()
        node.pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
