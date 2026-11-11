#!/usr/bin/env python3
"""速度指令安全监督节点。

订阅原始速度指令 /cmd_vel_raw，经过四道保护后发布到 /cmd_vel：
1. 限幅：线速度、角速度超过参数上限时截断；
2. 急停：/emergency_stop 为 true 时输出零速度并清除缓存；
3. 超时保护：超过 cmd_timeout 秒没有收到新指令时输出零速度；
4. 非法值拦截：指令含 NaN / Inf 时输出零速度。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _clamp(value, limit):
    """把 value 限制在 [-limit, limit] 内，非有限值返回 0.0。"""
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


def _is_finite_twist(msg):
    """判断 Twist 的六个分量是否都是有限值。"""
    values = (
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z,
    )
    return all(math.isfinite(v) for v in values)


class CmdVelGuard(Node):
    """限幅、急停、超时、非法值拦截四道保护的速度监督节点。"""

    def __init__(self):
        super().__init__('cmd_vel_guard')

        self.max_linear_x = self.declare_parameter(
            'max_linear_x', 0.3).value
        self.max_linear_y = self.declare_parameter(
            'max_linear_y', 0.0).value
        self.max_angular_z = self.declare_parameter(
            'max_angular_z', 0.8).value
        self.cmd_timeout = self.declare_parameter(
            'cmd_timeout', 2.0).value
        self.publish_rate = self.declare_parameter(
            'publish_rate', 20.0).value

        self.validate_parameters()

        self.last_cmd = None
        self.last_cmd_time = None
        self.estop = False
        self.last_status = None

        self.create_subscription(
            Twist, 'cmd_vel_raw', self.cmd_callback, 10)
        self.create_subscription(
            Bool, 'emergency_stop', self.estop_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, 'guard_status', 10)

        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.on_timer)
        self.get_logger().info(
            '速度监督已启动: max_linear_x=%.2f m/s, '
            'max_angular_z=%.2f rad/s, timeout=%.1f s'
            % (self.max_linear_x, self.max_angular_z, self.cmd_timeout))

    def validate_parameters(self):
        """检查参数合法性，非法时抛出 ValueError。"""
        limits = {
            'max_linear_x': self.max_linear_x,
            'max_linear_y': self.max_linear_y,
            'max_angular_z': self.max_angular_z,
        }
        for name, value in limits.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    '%s 必须是非负有限值，当前为 %r' % (name, value))
        if not math.isfinite(self.cmd_timeout) or self.cmd_timeout <= 0.0:
            raise ValueError(
                'cmd_timeout 必须大于 0，当前为 %r' % (self.cmd_timeout,))
        if (not math.isfinite(self.publish_rate)
                or self.publish_rate <= 0.0):
            raise ValueError(
                'publish_rate 必须大于 0，当前为 %r'
                % (self.publish_rate,))

    def cmd_callback(self, msg):
        if self.estop:
            return
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def estop_callback(self, msg):
        if msg.data and not self.estop:
            self.get_logger().warn('收到急停指令，已清除缓存指令并输出零速度')
            self.last_cmd = None
            self.last_cmd_time = None
        elif not msg.data and self.estop:
            self.get_logger().info('急停已解除，等待新的速度指令')
        self.estop = msg.data

    def publish_zero(self, status):
        self.cmd_pub.publish(Twist())
        self.publish_status(status)

    def publish_status(self, status):
        if status != self.last_status:
            self.get_logger().info('监督状态: %s' % status)
            self.last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def on_timer(self):
        if self.estop:
            self.publish_zero('estop')
            return

        if self.last_cmd is None:
            self.publish_zero('waiting')
            return

        if not _is_finite_twist(self.last_cmd):
            if self.last_status != 'invalid':
                self.get_logger().error(
                    '收到非法速度指令（NaN/Inf），已输出零速度')
            self.publish_zero('invalid')
            return

        elapsed = (
            self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            if self.last_status != 'timeout':
                self.get_logger().warn(
                    '%.1f 秒未收到速度指令，已自动停车' % elapsed)
            self.publish_zero('timeout')
            return

        cmd = Twist()
        cmd.linear.x = _clamp(self.last_cmd.linear.x, self.max_linear_x)
        cmd.linear.y = _clamp(self.last_cmd.linear.y, self.max_linear_y)
        cmd.angular.z = _clamp(
            self.last_cmd.angular.z, self.max_angular_z)

        clamped = (cmd.linear.x != self.last_cmd.linear.x
                   or cmd.linear.y != self.last_cmd.linear.y
                   or cmd.angular.z != self.last_cmd.angular.z)
        if clamped:
            self.get_logger().warn(
                '指令超限已截断: x %.2f->%.2f, z %.2f->%.2f' % (
                    self.last_cmd.linear.x, cmd.linear.x,
                    self.last_cmd.angular.z, cmd.angular.z),
                throttle_duration_sec=1.0)

        self.cmd_pub.publish(cmd)
        self.publish_status('clamped' if clamped else 'ok')


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('cmd_vel_guard')
    try:
        node = CmdVelGuard()
    except ValueError as exc:
        logger.fatal('参数非法，节点退出: %s' % exc)
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
