#!/usr/bin/env python3
"""上游心跳看门狗节点。

订阅 /heartbeat（std_msgs/String，内容是节点名），记录每个上游节点最后一次心跳时间，
按 watched_nodes 名单逐个检查：超时未上报的节点视为失联，输出状态并发布急停。

与 cmd_vel_guard 的超时保护不同：那个看的是「有没有新速度指令」，
本节点看的是「产生指令的节点是否还活着」，因此上游不动（指令恒为 0）时依然有效。
"""

import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _stale_nodes(last_seen, now_seconds, timeout, watched_nodes):
    """返回超时或从未上报的节点名列表。"""
    stale = []
    for name in watched_nodes:
        stamp = last_seen.get(name)
        if stamp is None:
            stale.append(name)
        elif now_seconds - stamp > timeout:
            stale.append(name)
    return stale


def _split_nodes(value):
    """把参数解析成节点名列表，支持逗号分隔的字符串或字符串列表。"""
    if isinstance(value, str):
        parts = value.split(',')
    elif isinstance(value, (list, tuple)):
        parts = [str(item) for item in value]
    else:
        return []
    return [part.strip() for part in parts if part.strip()]


class HeartbeatWatchdog(Node):
    """上游心跳看门狗节点。"""

    def __init__(self, **kwargs):
        super().__init__('heartbeat_watchdog', **kwargs)

        self.watched_nodes = _split_nodes(self.declare_parameter(
            'watched_nodes', 'navigator').value)
        self.heartbeat_timeout = float(self.declare_parameter(
            'heartbeat_timeout', 1.0,
            descriptor=ParameterDescriptor(
                dynamic_typing=True)).value)
        self.publish_rate = float(self.declare_parameter(
            'publish_rate', 10.0,
            descriptor=ParameterDescriptor(
                dynamic_typing=True)).value)
        self.publish_estop = self.declare_parameter(
            'publish_estop', True).value
        self.validate_parameters()

        self.last_seen = {}
        self.last_status = None

        self.create_subscription(String, 'heartbeat', self.heartbeat_callback,
                                 10)
        self.status_pub = self.create_publisher(
            String, 'watchdog_status', 10)
        self.estop_pub = self.create_publisher(
            Bool, 'emergency_stop', 10)
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            '心跳看门狗已启动: 监视 %s，超时 %.1f s'
            % (', '.join(self.watched_nodes), self.heartbeat_timeout))

    def validate_parameters(self):
        """检查参数合法性，非法时抛出 ValueError。"""
        nodes = self.watched_nodes
        if not isinstance(nodes, (list, tuple)) or len(nodes) == 0:
            raise ValueError('watched_nodes 必须是非空列表，当前为 %r'
                             % (nodes,))
        for name in nodes:
            if not isinstance(name, str) or not name.strip():
                raise ValueError(
                    'watched_nodes 中存在空名称，当前为 %r' % (nodes,))
        if (not math.isfinite(self.heartbeat_timeout)
                or self.heartbeat_timeout <= 0.0):
            raise ValueError(
                'heartbeat_timeout 必须大于 0，当前为 %r'
                % (self.heartbeat_timeout,))
        if (not math.isfinite(self.publish_rate)
                or self.publish_rate <= 0.0):
            raise ValueError(
                'publish_rate 必须大于 0，当前为 %r'
                % (self.publish_rate,))

    def heartbeat_callback(self, msg):
        name = msg.data.strip()
        if name and name in self.watched_nodes:
            self.last_seen[name] = self._now_seconds()

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_result(self, status, estop):
        if status != self.last_status:
            self.get_logger().info('看门狗状态: %s' % status)
            self.last_status = status

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        if self.publish_estop:
            estop_msg = Bool()
            estop_msg.data = bool(estop)
            self.estop_pub.publish(estop_msg)

    def on_timer(self):
        missing = [name for name in self.watched_nodes
                   if name not in self.last_seen]
        if missing:
            self.publish_result('waiting', True)
            return

        stale = _stale_nodes(
            self.last_seen, self._now_seconds(),
            self.heartbeat_timeout, self.watched_nodes)

        if stale:
            self.publish_result('stale: %s' % ', '.join(stale), True)
            return

        self.publish_result('ok', False)


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('heartbeat_watchdog')
    try:
        node = HeartbeatWatchdog()
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
