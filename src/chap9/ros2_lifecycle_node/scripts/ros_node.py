#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一个真正的 rclpy 生命周期节点：只有被 activate 之后才发布心跳。

    ros2 run ros2_lifecycle_node ros_node.py
    # 另开终端，按顺序操作：
    ros2 lifecycle get /lifecycle_heartbeat
    ros2 lifecycle set /lifecycle_heartbeat configure
    ros2 lifecycle set /lifecycle_heartbeat activate
    ros2 topic echo /heartbeat          # 现在才有数据
    ros2 lifecycle set /lifecycle_heartbeat deactivate
    ros2 lifecycle set /lifecycle_heartbeat cleanup

为什么用 lifecycle 而不是普通节点：configure 里做「可能失败的重活」（读参数、
连接传感器、建 publisher），失败就直接返回 FAILURE，节点停在 unconfigured，
不会带着半初始化的状态对外发消息。activate 只做「确定能成功的轻活」，
所以启停可以反复执行。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Float64, String


class HeartbeatNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_heartbeat')
        self.declare_parameter('period', 0.5)
        self.declare_parameter('topic', '/heartbeat')
        self.declare_parameter('fail_on_configure', False)

        self.pub = None
        self.state_pub = None
        self.timer = None
        self.count = 0
        self.get_logger().info('节点已构造，但还什么都没做（unconfigured）')

    # -- configure：建资源，可以失败 -------------------------------------
    def on_configure(self, state):
        if self.get_parameter('fail_on_configure').value:
            self.get_logger().error('configure 故意失败：停在 unconfigured')
            return TransitionCallbackReturn.FAILURE

        topic = self.get_parameter('topic').value
        self.pub = self.create_lifecycle_publisher(Float64, topic, 10)
        self.state_pub = self.create_lifecycle_publisher(
            String, '/lifecycle/report', 10)
        self.get_logger().info('configured：publisher 已创建（尚未激活）')
        return TransitionCallbackReturn.SUCCESS

    # -- activate：开始工作 ----------------------------------------------
    def on_activate(self, state):
        period = float(self.get_parameter('period').value)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info('activated：开始发布，周期 %.2f s' % period)
        return super().on_activate(state)

    # -- deactivate：停工作，但保留资源 -----------------------------------
    def on_deactivate(self, state):
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        self.get_logger().info('deactivated：已停止发布，累计 %d 条' % self.count)
        return super().on_deactivate(state)

    # -- cleanup：释放资源，回到 unconfigured ------------------------------
    def on_cleanup(self, state):
        if self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None
        if self.state_pub is not None:
            self.destroy_publisher(self.state_pub)
            self.state_pub = None
        self.get_logger().info('cleaned up：publisher 已释放')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('shutdown')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        self.get_logger().error('进入 error 处理')
        return TransitionCallbackReturn.SUCCESS

    # -- 定时任务 ---------------------------------------------------------
    def tick(self):
        msg = Float64()
        msg.data = float(self.count)
        self.pub.publish(msg)
        info = String()
        info.data = 'count=%d' % self.count
        self.state_pub.publish(info)
        self.count += 1


def main(argv=None):
    import rclpy
    rclpy.init(args=argv)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
