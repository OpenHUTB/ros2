"""
水下机器人键盘遥控节点
=====================
通过键盘控制 ROV 的 6-DOF 运动，发布 /cmd_vel 话题。

按键映射:
    W/S — 前进/后退 (X轴)
    A/D — 左移/右移 (Y轴)
    Q/E — 上浮/下潜 (Z轴)
    J/L — 左偏航/右偏航
    I/K — 前俯仰/后俯仰
    U/O — 左翻滚/右翻滚
    空格 — 急停（所有速度归零）
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# 按键到速度指令的映射 (支持方向键与数字键 1-9)
KEY_BINDINGS = {
    # 方向键 (Arrow Keys)
    'KEY_UP':    ('linear', 'x',  1.0),   # ↑ 前进
    'KEY_DOWN':  ('linear', 'x', -1.0),  # ↓ 后退
    'KEY_LEFT':  ('angular', 'z',  1.0),  # ← 原地左转
    'KEY_RIGHT': ('angular', 'z', -1.0),  # → 原地右转

    # 数字键 / 小键盘九宫格 (Numpad 1-9)
    '8': ('linear', 'z',  1.0),   # [8] 垂直上浮
    '2': ('linear', 'z', -1.0),  # [2] 垂直下潜
    '7': ('linear', 'y',  1.0),   # [7] 侧向左移
    '9': ('linear', 'y', -1.0),  # [9] 侧向右移
    '4': ('angular', 'z',  1.0),  # [4] 原地左转
    '6': ('angular', 'z', -1.0),  # [6] 原地右转
    '1': ('angular', 'y',  1.0),  # [1] 俯仰调节 (低头)
    '3': ('angular', 'y', -1.0),  # [3] 俯仰调节 (抬头)

    # 字母键兼容
    'w': ('linear', 'x',  1.0),   # 前进
    's': ('linear', 'x', -1.0),   # 后退
    'a': ('linear', 'y',  1.0),   # 左移
    'd': ('linear', 'y', -1.0),   # 右移
    'q': ('linear', 'z',  1.0),   # 上浮
    'e': ('linear', 'z', -1.0),   # 下潜
}

HELP_TEXT = """
╔═══════════════════════════════════════════════════════════╗
║         水下机器人 6-DOF 运动控制 (方向键与数字键)        ║
╠═══════════════════════════════════════════════════════════╣
║  【方向键】：                                             ║
║       ↑ : 前进 (Forward)                                  ║
║       ↓ : 后退 (Backward)                                 ║
║       ← : 原地左转 (Turn Left)    → : 原地右转 (Turn Right)║
║                                                           ║
║  【数字键 / 小键盘九宫格 (1-9)】：                        ║
║     [ 7 ] 左横移       [ 8 ] 垂直上浮       [ 9 ] 右横移  ║
║     [ 4 ] 原地左转     [ 5 ] 急停悬停       [ 6 ] 原地右转║
║     [ 1 ] 俯仰调节     [ 2 ] 垂直下潜       [ 3 ] 俯仰调节║
║                                                           ║
║  【通用操作】：                                           ║
║     [ 空格 ] 或 [ 0 ] / [ 5 ] : 急停并锁定当前水深        ║
║     [ + ] / [ - ]             : 增大 / 减小推力比例       ║
║     [ Ctrl + C ]              : 安全退出                  ║
╚═══════════════════════════════════════════════════════════╝
"""


class KeyboardTeleopNode(Node):
    """键盘遥控节点"""

    def __init__(self):
        super().__init__('keyboard_teleop_node')

        # 参数
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('publish_rate', 20.0)

        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # 发布者
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # 当前速度指令
        self.twist = Twist()

        # 定时发布
        self.timer = self.create_timer(1.0 / pub_rate, self._publish_callback)

        # 终端设置
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except Exception:
            self.settings = None

        self.get_logger().info('键盘遥控节点已启动 (支持方向键与数字键)')
        print(HELP_TEXT)

    def _get_key(self, timeout=0.05):
        """非阻塞读取键盘输入 (支持 ANSI 方向键转义序列)"""
        if self.settings is None:
            return ''
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            key = ''
            if rlist:
                c = sys.stdin.read(1)
                if c == '\x1b':
                    # 检测转义序列 \x1b[A, B, C, D
                    r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if r2:
                        c2 = sys.stdin.read(1)
                        if c2 == '[':
                            r3, _, _ = select.select([sys.stdin], [], [], 0.05)
                            if r3:
                                c3 = sys.stdin.read(1)
                                if c3 == 'A': key = 'KEY_UP'
                                elif c3 == 'B': key = 'KEY_DOWN'
                                elif c3 == 'C': key = 'KEY_RIGHT'
                                elif c3 == 'D': key = 'KEY_LEFT'
                else:
                    key = c
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except Exception:
            return ''

    def _publish_callback(self):
        """定时读取按键并发布速度指令"""
        key = self._get_key()

        if key in (' ', '0', '5'):
            # 急停并悬停
            self.twist = Twist()
            self.get_logger().info('急停锁定当前水深！')
        elif key in ('+', '='):
            self.linear_scale = min(5.0, self.linear_scale + 0.1)
            self.angular_scale = min(5.0, self.angular_scale + 0.1)
            self.get_logger().info(f'速度比例: 线={self.linear_scale:.1f} 角={self.angular_scale:.1f}')
        elif key == '-':
            self.linear_scale = max(0.1, self.linear_scale - 0.1)
            self.angular_scale = max(0.1, self.angular_scale - 0.1)
            self.get_logger().info(f'速度比例: 线={self.linear_scale:.1f} 角={self.angular_scale:.1f}')
        elif key in KEY_BINDINGS:
            group, axis, value = KEY_BINDINGS[key]
            self.twist = Twist()  # 单键触发

            if group == 'linear':
                scale = self.linear_scale
            else:
                scale = self.angular_scale

            setattr(getattr(self.twist, group), axis, value * scale)
        elif key == '\x03':
            # Ctrl+C
            raise KeyboardInterrupt

        self.pub_cmd_vel.publish(self.twist)

    def destroy_node(self):
        """恢复终端设置"""
        if self.settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
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
