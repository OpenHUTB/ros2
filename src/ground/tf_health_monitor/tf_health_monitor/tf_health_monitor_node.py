#!/usr/bin/env python3
"""TF 链路健康监视节点。

订阅 /tf 与 /tf_static，把每条 (parent, child) 边的观测聚合成频率与新鲜度，
输出总体状态与异常边清单。判据集中在 core.py，不依赖 ROS，可离线回归。

本节点只做判断，不修改 TF、不代发变换，因此可以直接挂到任意正在运行的
导航栈旁边使用。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from tf_health_monitor import core


def _string(data):
    """构造一个 std_msgs/String。"""
    msg = String()
    msg.data = data
    return msg


class TfHealthMonitor(Node):
    """TF 链路健康监视节点。"""

    def __init__(self):
        super().__init__('tf_health_monitor')

        self.stale_timeout = float(
            self.declare_parameter('stale_timeout', 0.5).value)
        self.rate_window = float(
            self.declare_parameter('rate_window', 2.0).value)
        self.min_rate = float(
            self.declare_parameter('min_rate', 1.0).value)
        self.min_samples = int(
            self.declare_parameter('min_samples', 5).value)
        self.backwards_hold = float(
            self.declare_parameter('backwards_hold', 2.0).value)
        self.stamp_lag = float(
            self.declare_parameter('stamp_lag', 1.0).value)
        self.publish_rate = float(
            self.declare_parameter('publish_rate', 5.0).value)
        # 逗号分隔的 parent->child 列表；空字符串表示动态发现所有边。
        watched_spec = str(self.declare_parameter('watched', '').value)
        self.watched = core.parse_watched_frames(watched_spec.split(','))

        core.validate_parameters(
            self.stale_timeout, self.rate_window, self.min_rate,
            self.min_samples, self.backwards_hold, self.stamp_lag,
            self.publish_rate)

        self.tracker = core.TfHealthTracker(
            stale_timeout=self.stale_timeout,
            rate_window=self.rate_window,
            min_rate=self.min_rate,
            min_samples=self.min_samples,
            backwards_hold=self.backwards_hold,
            stamp_lag=self.stamp_lag,
            watched=self.watched)
        self.last_status = None

        # /tf 的发布者可能是 reliable，也可能是 best_effort；订阅端用
        # best_effort 两种都能收到，反过来会匹配失败。
        tf_qos = QoSProfile(
            depth=100, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        # /tf_static 是 transient_local，晚加入的订阅者也能补到历史。
        static_qos = QoSProfile(
            depth=100, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(TFMessage, 'tf', self.tf_callback, tf_qos)
        self.create_subscription(
            TFMessage, 'tf_static', self.static_callback, static_qos)
        self.status_pub = self.create_publisher(String, 'tf_health', 10)
        self.edges_pub = self.create_publisher(String, 'tf_health_edges', 10)
        self.problem_pub = self.create_publisher(
            String, 'tf_health_problem', 10)
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.on_timer)

        watched_text = ('未声明，动态发现' if not self.watched
                        else ','.join('%s->%s' % key for key in self.watched))
        self.get_logger().info(
            'TF 健康监视已启动: stale>%.2f s, min_rate=%.2f Hz, '
            'window=%.1f s, 监视项=%s'
            % (self.stale_timeout, self.min_rate, self.rate_window,
               watched_text))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _ingest(self, msg, static):
        now = self._now()
        for transform in msg.transforms:
            header = transform.header
            stamp = header.stamp
            stamp_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
            self.tracker.observe(
                header.frame_id, transform.child_frame_id,
                stamp_ns, now, static=static)

    def tf_callback(self, msg):
        self._ingest(msg, static=False)

    def static_callback(self, msg):
        self._ingest(msg, static=True)

    def on_timer(self):
        reports = self.tracker.evaluate(self._now())
        status, problems = core.overall_status(reports)

        if status != self.last_status:
            if status == core.OK:
                self.get_logger().info(
                    'TF 链路正常: %d 条边, 采样 %d 条'
                    % (len(reports), self.tracker.total_samples))
            elif status == core.UNKNOWN:
                self.get_logger().info('TF 链路样本不足，等待更多数据')
            elif status == core.NO_INPUT:
                self.get_logger().warn(
                    '尚未收到 /tf 或 /tf_static，检查 TF 发布者是否在运行')
            else:
                self.get_logger().warn(
                    'TF 链路异常: %s -> %s'
                    % (status, core.format_problems(problems)))
            self.last_status = status

        self.status_pub.publish(_string(status))
        self.edges_pub.publish(_string(core.format_report(reports)))
        self.problem_pub.publish(_string(core.format_problems(problems)))


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('tf_health_monitor')
    try:
        node = TfHealthMonitor()
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
