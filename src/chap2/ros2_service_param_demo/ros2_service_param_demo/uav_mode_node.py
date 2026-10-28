#!/usr/bin/env python3
"""无人机模式节点：演示 ROS 2 的服务、参数与话题发布."""

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

VALID_MODES = ('IDLE', 'FLYING', 'LANDING')
MAX_PUBLISH_RATE = 100.0


class UavModeNode(Node):
    """提供服务 /uav/set_mode 与参数 publish_rate，并周期发布 /uav/status."""

    def __init__(self):
        """创建服务、参数与状态发布定时器."""
        super().__init__('uav_mode_node')

        # start_mode 只允许在启动时配置，运行中修改会被 rclpy 拒绝
        self.declare_parameter(
            'start_mode', 'IDLE',
            ParameterDescriptor(
                read_only=True, description='启动模式：IDLE / FLYING / LANDING'))
        self.declare_parameter(
            'publish_rate', 2.0,
            ParameterDescriptor(description='状态发布频率，范围 (0, 100] Hz'))

        self.publish_rate = self.read_publish_rate()
        self.mode = self.read_start_mode()
        self.count = 0

        # 参数更新回调：先校验、后生效
        self.add_on_set_parameters_callback(self.on_parameter_update)

        # 服务：切换飞行模式（true -> FLYING，false -> IDLE）
        self.create_service(SetBool, '/uav/set_mode', self.on_set_mode)

        # 话题：周期发布当前模式与计数
        self.publisher = self.create_publisher(String, '/uav/status', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_status)

        self.get_logger().info(
            'uav_mode_node 已启动: publish_rate=%.1f Hz, mode=%s'
            % (self.publish_rate, self.mode))

    def read_publish_rate(self):
        """读取并校验 publish_rate，非法值回退到 2.0，避免启动期除零."""
        value = float(self.get_parameter('publish_rate').value)
        if value <= 0.0 or value > MAX_PUBLISH_RATE:
            self.get_logger().warn(
                'publish_rate=%.3f 不在 (0, %.0f] 范围内，回退到 2.0 Hz'
                % (value, MAX_PUBLISH_RATE))
            return 2.0
        return value

    def read_start_mode(self):
        """读取并校验 start_mode，非法值回退到 IDLE."""
        value = self.get_parameter('start_mode').value
        if value not in VALID_MODES:
            self.get_logger().warn('start_mode=%s 不合法，回退到 IDLE' % value)
            return 'IDLE'
        return value

    def on_set_mode(self, request, response):
        """服务回调：根据请求把模式切换为 FLYING 或 IDLE."""
        self.mode = 'FLYING' if request.data else 'IDLE'
        response.success = True
        response.message = 'mode changed to ' + self.mode
        self.get_logger().info('模式切换为 %s' % self.mode)
        return response

    def on_parameter_update(self, parameters):
        """参数回调：先整体校验，全部合法后再应用到属性与定时器."""
        new_rate = None
        for param in parameters:
            if param.name == 'publish_rate':
                new_rate = float(param.value)

        if new_rate is not None and new_rate <= 0.0:
            return SetParametersResult(
                successful=False, reason='publish_rate 必须大于 0')
        if new_rate is not None and new_rate > MAX_PUBLISH_RATE:
            return SetParametersResult(
                successful=False,
                reason='publish_rate 不能超过 %.0f' % MAX_PUBLISH_RATE)

        if new_rate is not None:
            self.publish_rate = new_rate
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
        # ros2 launch 下 Ctrl+C 可能在销毁节点时再抛一次 KeyboardInterrupt
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
