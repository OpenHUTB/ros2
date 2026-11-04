#!/usr/bin/env python3
"""发布者节点：周期性地向 chatter 话题发布字符串消息."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """以 10 Hz 的频率向 chatter 话题发布消息的发布者节点."""

    def __init__(self):
        """创建发布者与定时器."""
        super().__init__('talker')
        # 发布者：话题 chatter，消息类型 String，队列长度 10
        self.publisher = self.create_publisher(String, 'chatter', 10)
        # 定时器：0.1 秒触发一次，即 10 Hz
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        """定时器回调：构造并发布一条消息."""
        msg = String()
        msg.data = 'Hello ROS2, count=%d' % self.count
        self.publisher.publish(msg)
        self.get_logger().info('发布: %s' % msg.data)
        self.count += 1


def main(args=None):
    """启动 talker 节点并保持运行，直到收到 Ctrl+C."""
    rclpy.init(args=args)
    node = TalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('talker 节点被手动终止')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
