#!/usr/bin/env python3
"""TF 演示数据源：按固定时间表复现四类 TF 异常。

配合 tf_health_monitor 使用，不需要底盘、不需要仿真器。时间表（秒）：

1. ``0–8``   正常发布 ``map->odom`` 与 ``odom->base_footprint``；
2. ``8–18``  停止发布 ``odom->base_footprint``，复现发布者掉线（stale）；
3. ``18–26`` 恢复发布，但时间戳停在 8 s 前，复现 TF_OLD_DATA（old_data）；
4. ``26 以后`` 再挂一个 ``world->base_footprint``，复现多 parent（multi_parent）。

``base_link->laser`` 通过 ``/tf_static`` 只发一次，用来验证静态边不会被判成断链。
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_msgs.msg import TFMessage

PHASE_ONE = 8.0
PHASE_TWO = 18.0
PHASE_THREE = 26.0

PHASE_TEXT = {
    1: '1/4 正常发布（预期 ok）',
    2: '2/4 odom->base_footprint 停发（预期 stale）',
    3: '3/4 时间戳停在 10 s 前恢复（预期 old_data）',
    4: '4/4 再出现一个 parent（预期 multi_parent）',
}


class TfDemoPublisher(Node):
    """按时间表制造 TF 异常的演示发布者。"""

    def __init__(self):
        super().__init__('tf_demo_publisher')
        self.publish_rate = float(
            self.declare_parameter('publish_rate', 20.0).value)
        if self.publish_rate <= 0.0:
            raise ValueError(
                'publish_rate 必须大于 0，当前为 %r' % (self.publish_rate,))

        tf_qos = QoSProfile(
            depth=10, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_pub = self.create_publisher(TFMessage, 'tf', tf_qos)
        self.static_pub = self.create_publisher(
            TFMessage, 'tf_static', static_qos)

        self.start_time = self._now()
        self.phase = None
        self.edge_stamp = None
        self.static_pub.publish(TFMessage(
            transforms=[self._build('base_link', 'laser', self._now())]))
        self.get_logger().info('已发布静态边 base_link->laser')
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.on_timer)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _build(self, parent, child, seconds):
        """构造一条变换，时间戳由 seconds 指定而不是取当前时刻。"""
        item = TransformStamped()
        item.header.frame_id = parent
        item.child_frame_id = child
        total = int(round(seconds * 1e9))
        item.header.stamp.sec = total // 1000000000
        item.header.stamp.nanosec = total % 1000000000
        item.transform.rotation.w = 1.0
        return item

    def on_timer(self):
        elapsed = self._now() - self.start_time
        phase = self._phase_of(elapsed)
        if phase != self.phase:
            self.phase = phase
            self.get_logger().info('阶段 %s' % PHASE_TEXT[phase])

        now = self._now()
        step = 1.0 / self.publish_rate
        transforms = [self._build('map', 'odom', now)]

        if phase == 2:
            # 这一段不发布 odom->base_footprint，让它的观测自然过期。
            pass
        elif phase == 3:
            if self.edge_stamp is None:
                # 从停发前的最后一帧继续，保证不触发「时间戳回退」，
                # 只暴露「消息陈旧」这一种异常。
                self.edge_stamp = now - 10.0
            transforms.append(
                self._build('odom', 'base_footprint', self.edge_stamp))
            self.edge_stamp += step
        else:
            self.edge_stamp = None
            transforms.append(self._build('odom', 'base_footprint', now))

        if phase == 4:
            transforms.append(self._build('world', 'base_footprint', now))

        msg = TFMessage()
        msg.transforms = transforms
        self.tf_pub.publish(msg)

    def _phase_of(self, elapsed):
        if elapsed < PHASE_ONE:
            return 1
        if elapsed < PHASE_TWO:
            return 2
        if elapsed < PHASE_THREE:
            return 3
        return 4


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('tf_demo_publisher')
    try:
        node = TfDemoPublisher()
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
