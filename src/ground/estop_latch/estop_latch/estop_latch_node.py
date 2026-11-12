#!/usr/bin/env python3
"""急停闩锁节点。

订阅 /emergency_stop（std_msgs/Bool，可同时接多个安全模块），一旦收到 true
就把急停闩锁下来：即使信号随后消失，输出仍然保持 true，必须通过 /estop_latch/reset
服务人工确认后才放行。默认上电即闩锁，避免「上游还没起来，底盘先动」。

与 heartbeat_watchdog 的区别：那个判断上游节点是否存活，本节点负责把已经发生的
安全事件锁住，交给人工确认，防止瞬态故障自我恢复后底盘在无人察觉时重新运动。
"""
import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def evaluate_release(now_seconds, last_input_time, last_active_time,
                     input_active, input_timeout, release_hold_time):
    """判断能否复位，返回 (是否可以, 原因)。"""
    if last_input_time is None:
        return False, '尚未收到 emergency_stop 输入'
    if input_active:
        return False, '急停信号仍处于触发状态'
    if input_timeout > 0.0 and now_seconds - last_input_time > input_timeout:
        return False, '输入链路 %.1f s 未更新' % input_timeout
    if now_seconds - last_active_time < release_hold_time:
        return False, '急停信号消失不足 %.1f s' % release_hold_time
    return True, ''


def input_stale(now_seconds, last_input_time, input_timeout):
    """输入链路是否已经超时；超时视为无法确认安全。"""
    if input_timeout <= 0.0:
        return False
    if last_input_time is None:
        return True
    return now_seconds - last_input_time > input_timeout


class EstopLatch(Node):
    """急停闩锁节点：锁住急停，人工复位后才放行。"""

    def __init__(self, **kwargs):
        super().__init__('estop_latch', **kwargs)
        self.input_timeout = float(self.declare_parameter(
            'input_timeout', 1.0,
            descriptor=ParameterDescriptor(dynamic_typing=True)).value)
        self.release_hold_time = float(self.declare_parameter(
            'release_hold_time', 2.0,
            descriptor=ParameterDescriptor(dynamic_typing=True)).value)
        self.status_rate = float(self.declare_parameter(
            'status_rate', 1.0,
            descriptor=ParameterDescriptor(dynamic_typing=True)).value)
        self.auto_release = self.declare_parameter(
            'auto_release', False).value
        self.start_latched = self.declare_parameter(
            'start_latched', True).value
        self.validate_parameters()

        self.latched = bool(self.start_latched)
        self.latch_reason = 'start' if self.latched else ''
        self.last_input_time = None
        self.last_active_time = 0.0
        self.input_active = False

        self.create_subscription(
            Bool, 'emergency_stop', self.estop_callback, 10)
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)
        self.status_pub = self.create_publisher(
            String, 'estop_status', 10)
        self.create_service(Trigger, '~/reset', self.handle_reset)
        self.timer = self.create_timer(
            1.0 / self.status_rate, self.on_timer)
        self.get_logger().info(
            '急停闩锁已启动: 保持 %.1f s 后可复位，启动状态 %s'
            % (self.release_hold_time,
               '闩锁' if self.latched else '放行'))

    def validate_parameters(self):
        """检查参数合法性，非法时抛出 ValueError。"""
        if (not math.isfinite(self.input_timeout)
                or self.input_timeout < 0.0):
            raise ValueError(
                'input_timeout 不能为负数，当前为 %r'
                % (self.input_timeout,))
        if (not math.isfinite(self.release_hold_time)
                or self.release_hold_time < 0.0):
            raise ValueError(
                'release_hold_time 不能为负数，当前为 %r'
                % (self.release_hold_time,))
        if (not math.isfinite(self.status_rate)
                or self.status_rate <= 0.0):
            raise ValueError(
                'status_rate 必须大于 0，当前为 %r' % (self.status_rate,))
        if not isinstance(self.auto_release, bool):
            raise ValueError(
                'auto_release 必须是布尔值，当前为 %r'
                % (self.auto_release,))
        if not isinstance(self.start_latched, bool):
            raise ValueError(
                'start_latched 必须是布尔值，当前为 %r'
                % (self.start_latched,))

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def estop_callback(self, msg):
        now = self._now_seconds()
        self.last_input_time = now
        self.input_active = bool(msg.data)
        if self.input_active:
            self.last_active_time = now
            if not self.latched:
                self.latched = True
                self.latch_reason = 'input'
                self.get_logger().warn('收到急停信号，进入闩锁')

    def release(self):
        self.latched = False
        self.latch_reason = ''
        self.get_logger().info('急停解除，允许运动')

    def handle_reset(self, request, response):
        ok, reason = evaluate_release(
            self._now_seconds(), self.last_input_time,
            self.last_active_time, self.input_active,
            self.input_timeout, self.release_hold_time)
        if not ok:
            response.success = False
            response.message = '拒绝复位：%s' % reason
            self.get_logger().warn('复位被拒绝: %s' % reason)
            return response
        self.release()
        response.success = True
        response.message = '已复位，急停解除'
        return response

    def publish_result(self):
        estop_msg = Bool()
        estop_msg.data = bool(self.latched)
        self.estop_pub.publish(estop_msg)
        status_msg = String()
        if self.latched:
            status_msg.data = 'latched: %s' % self.latch_reason
        else:
            status_msg.data = 'clear'
        self.status_pub.publish(status_msg)

    def on_timer(self):
        now = self._now_seconds()
        if not self.latched and input_stale(
                now, self.last_input_time, self.input_timeout):
            self.latched = True
            self.latch_reason = 'input_timeout'
            self.get_logger().error('未收到急停输入，按失效安全进入闩锁')
        elif self.latched and self.auto_release:
            ok, _ = evaluate_release(
                now, self.last_input_time, self.last_active_time,
                self.input_active, self.input_timeout,
                self.release_hold_time)
            if ok:
                self.release()
        self.publish_result()


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('estop_latch')
    try:
        node = EstopLatch()
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
