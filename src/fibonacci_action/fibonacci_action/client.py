#!/usr/bin/env python3
"""
ROS 2 Action 客户端：发送斐波那契计算请求并接收反馈与结果

运行方式：
    ros2 run fibonacci_action client
    ros2 run fibonacci_action client 15
或通过 launch 文件自动启动（order 可通过命令行参数修改）。
"""

import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from fibonacci_action.action import Fibonacci


class FibonacciActionClient(Node):
    """斐波那契数列计算 Action 客户端节点。"""

    def __init__(self, order: int = 10):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self._target_order = order
        self._finished = False
        self._exit_code = 0
        self.get_logger().info(f'客户端启动，目标阶数: {order}')

    def send_goal(self):
        """发送计算请求给服务端。"""
        goal_msg = Fibonacci.Goal()
        goal_msg.order = self._target_order

        self._action_client.wait_for_server()
        self.get_logger().info(
            f'已连接服务端，发送 goal (order={self._target_order})'
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """服务端响应 goal 时的回调。"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            # goal 被拒绝时不能直接 return，否则主循环会一直等待，
            # 客户端进程挂住不退出。
            self.get_logger().warn('Goal 被服务端拒绝，客户端退出')
            self._exit_code = 1
            self._finished = True
            return

        self.get_logger().info('Goal 已接受，等待结果...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """接收服务端反馈时的回调。"""
        sequence = feedback_msg.feedback.sequence
        self.get_logger().info(
            f'收到反馈: 当前数列长度={len(sequence)}, 最新项={sequence[-1]}'
        )

    def get_result_callback(self, future):
        """接收最终结果时的回调。"""
        wrapped = future.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'最终结果: {wrapped.result.sequence}')
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('目标已被取消，没有最终结果')
        else:
            self.get_logger().warn(f'目标未成功完成，状态码={wrapped.status}')

        # 不在回调里调用 rclpy.shutdown()：
        # 回调执行期间销毁上下文会抛 ExternalShutdownException 并打印 traceback。
        # 这里只置标志，由 main() 的主循环统一退出。
        self._finished = True


def main(args=None):
    rclpy.init(args=args)

    # 支持命令行参数指定阶数：ros2 run fibonacci_action client 15
    order = 10
    if len(sys.argv) > 1:
        try:
            order = int(sys.argv[1])
        except ValueError:
            print(f'参数 "{sys.argv[1]}" 不是有效整数，使用默认值 10')

    node = FibonacciActionClient(order=order)
    node.send_goal()

    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，客户端退出')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(node._exit_code)


if __name__ == '__main__':
    main()
