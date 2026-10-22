"""键盘运动控制节点（ROS 2）。

通过终端原始输入读取按键，以增量方式改变线速度与角速度，并周期性
发布到 ``cmd_vel``。采用增量式控制（松开按键保持当前速度）更接近真实
遥控器手感，也便于长距离巡航。

按键：
    w/s 前进/后退   a/d 左转/右转
    q/e 前进+左转/右转   z/c 后退+左转/右转
    空格 紧急停止        x 退出
    +/- 调整速度上限

无交互式终端时（例如通过 launch 后台启动），自动切换为匀速直行模式，
保证仿真仍可运行并产出轨迹数据。
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP_TEXT = """
============================================================
        Gazebo 差速驱动小车 —— 键盘运动控制
============================================================
   移动控制                     速度调节
     w : 前进                     + : 提高线速度上限
     s : 后退                     - : 降低线速度上限
     a : 左转
     d : 右转                     其他
     q : 前进 + 左转              ------
     e : 前进 + 右转               空格 : 紧急停止
     z : 后退 + 左转                 x  : 退出程序
     c : 后退 + 右转
   --------------------------------------------------------
   提示：松开按键后车辆保持当前速度，需按空格停车。
============================================================
"""


class TeleopKeyboardNode(Node):
    """键盘遥控节点。"""

    def __init__(self):
        """声明参数并创建发布者与定时器。"""
        super().__init__('teleop_keyboard')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('angular_step', 0.2)
        self.declare_parameter('linear_max', 0.8)
        self.declare_parameter('angular_max', 1.2)
        self.declare_parameter('publish_rate', 20.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.linear_max = float(self.get_parameter('linear_max').value)
        self.angular_max = float(self.get_parameter('angular_max').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.linear = 0.0
        self.angular = 0.0
        self.limit_linear = self.linear_max
        self.limit_angular = self.angular_max

        self.settings = None
        self.has_tty = sys.stdin.isatty()
        self.tick = 0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        if not self.has_tty:
            self.get_logger().warn(
                '未检测到交互式终端，切换为自动巡航模式（持续直行）')

    def start_terminal(self):
        """切换终端为原始模式并打印帮助。"""
        if not self.has_tty:
            return
        try:
            self.settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except termios.error as exc:
            self.get_logger().warn('无法设置终端原始模式：%s' % exc)
            self.has_tty = False
            return
        print(HELP_TEXT)

    def restore_terminal(self):
        """恢复终端设置。"""
        if self.has_tty and self.settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except termios.error:
                pass

    # ------------------------------------------------------------------

    def read_key(self):
        """非阻塞读取一个字符；无输入时返回 None。"""
        try:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                return sys.stdin.read(1)
        except (select.error, IOError):
            return None
        return None

    def handle_key(self, key):
        """按键映射到速度变化。"""
        if key == 'w':
            self.linear += self.linear_step
        elif key == 's':
            self.linear -= self.linear_step
        elif key == 'a':
            self.angular += self.angular_step
        elif key == 'd':
            self.angular -= self.angular_step
        elif key == 'q':
            self.linear += self.linear_step
            self.angular += self.angular_step
        elif key == 'e':
            self.linear += self.linear_step
            self.angular -= self.angular_step
        elif key == 'z':
            self.linear -= self.linear_step
            self.angular += self.angular_step
        elif key == 'c':
            self.linear -= self.linear_step
            self.angular -= self.angular_step
        elif key == ' ':
            self.linear = 0.0
            self.angular = 0.0
            self.get_logger().info('紧急停止')
        elif key in ('+', '='):
            self.limit_linear = min(self.limit_linear + self.linear_step,
                                    self.linear_max)
            self.get_logger().info('线速度上限 → %.2f m/s' % self.limit_linear)
        elif key == '-':
            self.limit_linear = max(self.limit_linear - self.linear_step,
                                    self.linear_step)
            self.get_logger().info('线速度上限 → %.2f m/s' % self.limit_linear)
        elif key in ('x', '\x03'):
            self.get_logger().info('收到退出指令')
            self.restore_terminal()
            rclpy.shutdown()
            return

        self.linear = max(-self.limit_linear, min(self.linear, self.limit_linear))
        self.angular = max(-self.limit_angular,
                           min(self.angular, self.limit_angular))

    def on_timer(self):
        """周期回调：读键 → 更新速度 → 发布。"""
        if self.has_tty:
            key = self.read_key()
            if key is not None:
                self.handle_key(key)
        elif self.tick == 0:
            self.linear = self.linear_step * 2

        self.tick += 1

        if self.has_tty and self.tick % 10 == 0:
            sys.stdout.write(
                '\r  [速度] v = %+.2f m/s   ω = %+.2f rad/s  上限 %.2f | %.2f   '
                % (self.linear, self.angular,
                   self.limit_linear, self.limit_angular))
            sys.stdout.flush()

        twist = Twist()
        twist.linear.x = self.linear
        twist.angular.z = self.angular
        self.cmd_pub.publish(twist)


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    node.start_terminal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
