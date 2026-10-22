#!/usr/bin/env python3
"""无人机模式节点：演示 ROS 2 的服务、参数与话题发布."""

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class UavModeNode(Node):
    """提供服务 /uav/set_mode 与参数 publish_rate，并周期发布 /uav/status."""

    def __init__(self):
        """创建服务、参数与状态发布定时器."""
        super().__init__('uav_mode_node')

        # 参数：先声明默认值，之后可用 ros2 param set 在线修改
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('start_mode', 'IDLE')

        self.publish_rate = self.get_parameter('publish_rate').value
        self.mode = self.get_parameter('start_mode').value
        self.count = 0

        # 参数更新回调：publish_rate 改变后立即按新频率重建定时器
        self.add_on_set_parameters_callback(self.on_parameter_update)

        # 服务：切换飞行模式（true -> FLYING，false -> IDLE）
        self.create_service(SetBool, '/uav/set_mode', self.on_set_mode)

        # 话题：周期发布当前模式与计数
        self.publisher = self.create_publisher(String, '/uav/status', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_status)

        self.get_logger().info(
            'uav_mode_node 已启动: publish_rate=%.1f Hz, mode=%s'
            % (self.publish_rate, self.mode))

    def on_set_mode(self, request, response):
        """服务回调：根据请求把模式切换为 FLYING 或 IDLE."""
        self.mode = 'FLYING' if request.data else 'IDLE'
        response.success = True
        response.message = 'mode changed to ' + self.mode
        self.get_logger().info('模式切换为 %s' % self.mode)
        return response

    def on_parameter_update(self, parameters):
        """参数回调：只接受大于 0 的 publish_rate，并让它立即生效."""
        rate_changed = False
        for param in parameters:
            if param.name == 'publish_rate':
                if param.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='publish_rate 必须大于 0')
                self.publish_rate = param.value
                rate_changed = True

        if rate_changed:
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_status)
            self.get_logger().info('publish_rate 已改为 %.1f Hz' % self.publish_rate)

        return SetParametersResult(successful=True)

    def publish_status(self):
        """定时器回调：发布一条状态消息."""
        msg = String()
        msg.data = 'mode=%s, count=%d' % (self.mode, self.count)
        self.publisher.publish(msg)
        self.get_logger().info('发布状态: %s' % msg.data)
        self.count += 1


def main(args=None):
    """启动节点并保持运行，直到收到 Ctrl+C."""
    rclpy.init(args=args)
    node = UavModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('uav_mode_node 被手动终止')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
