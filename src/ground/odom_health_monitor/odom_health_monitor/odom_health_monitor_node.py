#!/usr/bin/env python3
"""里程计健康监视节点。

同时订阅速度指令 /cmd_vel 与里程计 /odom，比较「期望速度」与「实测速度」：
1. ok：实测速度与期望速度的比例在正常区间内；
2. slip：实测速度明显大于期望速度，常见于轮子打滑或编码器放大；
3. stuck：实测速度接近 0 且持续超过 stuck_duration，常见于堵转或驱动失效；
4. reverse：实测运动方向与指令方向相反，常见于电机接线反接；
5. idle：指令速度在死区内（未要求底盘运动），不做判断；
6. no_cmd / no_odom：对应话题缺失或超时。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String


def _planar_speed(twist):
    """返回 Twist 的平面线速度模长。"""
    return math.hypot(twist.linear.x, twist.linear.y)


def _speed_ratio(measured, commanded, deadband):
    """实测与期望速度之比；期望速度在死区内时返回 None。"""
    if commanded <= deadband:
        return None
    return measured / commanded


def _is_reverse(cmd, odom_twist, deadband):
    """实测线速度与指令方向相反时返回 True。"""
    cmd_x, cmd_y = cmd.linear.x, cmd.linear.y
    odo_x = odom_twist.linear.x
    odo_y = odom_twist.linear.y
    if math.hypot(cmd_x, cmd_y) <= deadband:
        return False
    if math.hypot(odo_x, odo_y) <= deadband:
        return False
    return cmd_x * odo_x + cmd_y * odo_y < 0.0


class OdomHealthMonitor(Node):
    """里程计健康监视节点。"""

    def __init__(self):
        super().__init__('odom_health_monitor')

        self.velocity_deadband = self.declare_parameter(
            'velocity_deadband', 0.02).value
        self.stuck_ratio = self.declare_parameter(
            'stuck_ratio', 0.1).value
        self.slip_ratio = self.declare_parameter(
            'slip_ratio', 1.5).value
        self.stuck_duration = self.declare_parameter(
            'stuck_duration', 0.5).value
        self.cmd_timeout = self.declare_parameter(
            'cmd_timeout', 0.5).value
        self.odom_timeout = self.declare_parameter(
            'odom_timeout', 0.5).value
        self.publish_rate = self.declare_parameter(
            'publish_rate', 10.0).value
        self.validate_parameters()

        self.last_cmd = None
        self.last_cmd_time = None
        self.last_odom = None
        self.last_odom_time = None
        self.stuck_since = None
        self.last_status = None

        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.health_pub = self.create_publisher(
            String, 'odom_health', 10)
        self.ratio_pub = self.create_publisher(
            Float32, 'slip_ratio', 10)
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            '里程计监视已启动: deadband=%.2f m/s, stuck<%.2f, '
            'slip>%.2f, stuck_duration=%.1f s'
            % (self.velocity_deadband, self.stuck_ratio,
               self.slip_ratio, self.stuck_duration))

    def validate_parameters(self):
        """检查参数合法性，非法时抛出 ValueError。"""
        if (not math.isfinite(self.velocity_deadband)
                or self.velocity_deadband < 0.0):
            raise ValueError(
                'velocity_deadband 不能为负数，当前为 %r'
                % (self.velocity_deadband,))
        if (not math.isfinite(self.stuck_ratio)
                or self.stuck_ratio < 0.0 or self.stuck_ratio >= 1.0):
            raise ValueError(
                'stuck_ratio 必须在 [0, 1) 内，当前为 %r'
                % (self.stuck_ratio,))
        if (not math.isfinite(self.slip_ratio)
                or self.slip_ratio <= 1.0):
            raise ValueError(
                'slip_ratio 必须大于 1，当前为 %r'
                % (self.slip_ratio,))
        if (not math.isfinite(self.stuck_duration)
                or self.stuck_duration <= 0.0):
            raise ValueError(
                'stuck_duration 必须大于 0，当前为 %r'
                % (self.stuck_duration,))
        if (not math.isfinite(self.cmd_timeout)
                or self.cmd_timeout <= 0.0):
            raise ValueError(
                'cmd_timeout 必须大于 0，当前为 %r'
                % (self.cmd_timeout,))
        if (not math.isfinite(self.odom_timeout)
                or self.odom_timeout <= 0.0):
            raise ValueError(
                'odom_timeout 必须大于 0，当前为 %r'
                % (self.odom_timeout,))
        if (not math.isfinite(self.publish_rate)
                or self.publish_rate <= 0.0):
            raise ValueError(
                'publish_rate 必须大于 0，当前为 %r'
                % (self.publish_rate,))

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_time = self.get_clock().now()

    def _age(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def publish_result(self, status, ratio):
        if status != self.last_status:
            self.get_logger().info('里程计状态: %s' % status)
            self.last_status = status

        health_msg = String()
        health_msg.data = status
        self.health_pub.publish(health_msg)

        ratio_msg = Float32()
        ratio_msg.data = float(ratio)
        self.ratio_pub.publish(ratio_msg)

    def on_timer(self):
        now = self.get_clock().now()

        if self.last_odom is None:
            self.publish_result('no_odom', -1.0)
            return
        if self._age(self.last_odom_time) > self.odom_timeout:
            self.publish_result('no_odom', -1.0)
            return

        if self.last_cmd is None:
            self.publish_result('no_cmd', -1.0)
            return
        if self._age(self.last_cmd_time) > self.cmd_timeout:
            self.publish_result('no_cmd', -1.0)
            return

        measured = _planar_speed(self.last_odom.twist.twist)
        commanded = _planar_speed(self.last_cmd)
        ratio = _speed_ratio(measured, commanded, self.velocity_deadband)

        if ratio is None:
            self.stuck_since = None
            self.publish_result('idle', -1.0)
            return

        if _is_reverse(self.last_cmd, self.last_odom.twist.twist,
                       self.velocity_deadband):
            self.stuck_since = None
            self.publish_result('reverse', ratio)
            return

        if ratio > self.slip_ratio:
            self.stuck_since = None
            self.publish_result('slip', ratio)
            return

        if ratio < self.stuck_ratio:
            if self.stuck_since is None:
                self.stuck_since = now
            elapsed = (now - self.stuck_since).nanoseconds / 1e9
            if elapsed >= self.stuck_duration:
                self.publish_result('stuck', ratio)
            else:
                self.publish_result('ok', ratio)
            return

        self.stuck_since = None
        self.publish_result('ok', ratio)


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('odom_health_monitor')
    try:
        node = OdomHealthMonitor()
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
