"""
ROS 2 Action 服务端：斐波那契数列计算

接收客户端的请求（目标阶数 order），逐步计算斐波那契数列，
每算出一步就通过 Feedback 发布当前进度，完成后返回完整数列作为 Result。

运行方式：
    ros2 run fibonacci_action server
或通过 launch 文件自动启动。
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fibonacci_action.action import Fibonacci


class FibonacciActionServer(Node):
    """斐波那契数列计算 Action 服务端节点。"""

    def __init__(self):
        super().__init__('fibonacci_action_server')

        # 使用可重入回调组，允许同时处理多个 goal
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('✅ Fibonacci Action 服务端已就绪，等待客户端请求...')

    def goal_callback(self, goal_request):
        """接收 goal 请求时的回调，可用于参数校验。"""
        order = goal_request.order
        if order < 0 or order > 50:
            self.get_logger().warn(
                f'❌ 拒绝请求：order={order} 超出合法范围 [0, 50]'
            )
            return GoalResponse.REJECT

        self.get_logger().info(f'📥 接受请求：计算 {order} 阶斐波那契数列')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """接收取消请求时的回调。"""
        self.get_logger().info('🚫 收到取消请求')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """执行实际的计算逻辑，支持取消和反馈。"""
        self.get_logger().info(
            f'🔧 开始计算 {goal_handle.request.order} 阶斐波那契数列'
        )

        # 初始化数列
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('⛔ 任务已取消')
                return Fibonacci.Result()

            # 计算下一项
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i - 1]
            )

            # 发布反馈
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(
                f'📊 进度 {i + 1}/{goal_handle.request.order + 1}: '
                f'{feedback_msg.sequence}'
            )

            # 模拟计算耗时，便于观察反馈效果
            time.sleep(0.3)

        # 正常完成
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(
            f'🎉 计算完成！结果: {result.sequence}'
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer()

    # 使用多线程执行器，支持可重入回调组
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
