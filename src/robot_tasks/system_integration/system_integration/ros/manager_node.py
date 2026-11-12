"""集成层 ROS 节点：订阅两路速度，仲裁后发布唯一的 /cmd_vel。

话题：
    订阅  /cmd_vel_manual   任务1 键盘遥操作（launch 里由 /cmd_vel 重映射而来）
    订阅  /cmd_vel_auto     任务4 策略网络输出
    订阅  /integration/mode std_msgs/String，manual 或 auto
    订阅  /integration/estop std_msgs/Bool
    发布  /cmd_vel          唯一的速度出口
    发布  /integration/status std_msgs/String，一行文本
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from system_integration.arbiter import VelocityArbiter


class ManagerNode(Node):

    def __init__(self):
        super().__init__('integration_manager')

        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('mode', 'auto')
        self.declare_parameter('topic.manual', '/cmd_vel_manual')
        self.declare_parameter('topic.auto', '/cmd_vel_auto')
        self.declare_parameter('topic.cmd', '/cmd_vel')
        self.declare_parameter('topic.mode', '/integration/mode')
        self.declare_parameter('topic.estop', '/integration/estop')
        self.declare_parameter('topic.status', '/integration/status')

        self.arb = VelocityArbiter(
            timeout=float(self.get_parameter('timeout').value),
            max_linear=float(self.get_parameter('max_linear').value),
            max_angular=float(self.get_parameter('max_angular').value))

        # 启动模式由参数决定，默认 auto，配合 launch 默认启动的策略节点
        init_mode = str(self.get_parameter('mode').value)
        if not self.arb.set_mode(init_mode):
            self.get_logger().warn('参数 mode 非法：%s，沿用 manual' % init_mode)

        self.create_subscription(
            Twist, str(self.get_parameter('topic.manual').value),
            self.on_manual, 10)
        self.create_subscription(
            Twist, str(self.get_parameter('topic.auto').value),
            self.on_auto, 10)
        self.create_subscription(
            String, str(self.get_parameter('topic.mode').value),
            self.on_mode, 10)
        self.create_subscription(
            Bool, str(self.get_parameter('topic.estop').value),
            self.on_estop, 10)

        self.pub_cmd = self.create_publisher(
            Twist, str(self.get_parameter('topic.cmd').value), 10)
        self.pub_status = self.create_publisher(
            String, str(self.get_parameter('topic.status').value), 10)

        hz = max(1.0, float(self.get_parameter('rate_hz').value))
        self.create_timer(1.0 / hz, self.tick)
        self.last_mode = self.arb.mode
        self.get_logger().info(
            '集成层就绪：模式 %s，超时 %.2f s，限幅 %.2f m/s / %.2f rad/s，%.1f Hz'
            % (self.arb.mode, self.arb.timeout, self.arb.max_linear,
               self.arb.max_angular, hz))

    # ---------- 回调 ----------
    def on_manual(self, msg: Twist):
        self.arb.feed_manual(msg.linear.x, msg.angular.z)

    def on_auto(self, msg: Twist):
        self.arb.feed_auto(msg.linear.x, msg.angular.z)

    def on_mode(self, msg: String):
        text = msg.data.strip()
        if self.arb.set_mode(text):
            self.get_logger().info('模式切换 -> %s' % self.arb.mode)
        else:
            self.get_logger().warn('无法识别的模式：%s' % text)

    def on_estop(self, msg: Bool):
        self.arb.set_estop(msg.data)
        self.get_logger().warn('急停 %s' % ('触发' if msg.data else '解除'))

    # ---------- 主循环 ----------
    def tick(self):
        v, w, st = self.arb.step()

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

        if st['mode'] != self.last_mode:
            self.last_mode = st['mode']

        status = String()
        status.data = ('mode=%s estop=%s source=%s reason=%s age=%.3f'
                       % (st['mode'], int(st['estop']), st['source'],
                          st['reason'], st['age']))
        self.pub_status.publish(status)

    def stop(self):
        try:
            self.pub_cmd.publish(Twist())
        except Exception:
            pass


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = ManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
