#!/usr/bin/env python3
"""订阅者节点：接收并打印 chatter 话题上的字符串消息。"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """订阅 chatter 话题并打印收到的消息的订阅者节点。"""

    def __init__(self):
        """创建订阅者。"""
        super().__init__('listener')
        # 订阅者：话题 chatter，消息类型 String，队列长度 10
        self.subscription = self.create_subscription(
            String, 'chatter', self.callback, 10)

    def callback(self, msg):
        """订阅回调：打印收到的消息内容。"""
        self.get_logger().info('收到消息: %s' % msg.data)


def main(args=None):
    """启动 listener 节点并保持运行，直到收到 Ctrl+C。"""
    rclpy.init(args=args)
    node = ListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('listener 节点被手动终止')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
