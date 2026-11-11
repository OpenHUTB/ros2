"""ROS 2 节点：按参数选择的离散化方法，实时跑一遍二阶系统阶跃响应。

话题
----
- ``/control_discretization/y``（``std_msgs/Float64``）：每个采样周期的输出
- ``/control_discretization/metrics``（``std_msgs/String``）：跑完一轮后的误差与极点

参数：``method``（forward/backward/tustin/zoh）、``ts``、``wn``、``zeta``、``duration``。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .systems import (METHOD_LABELS, METHODS, SecondOrderSystem,
                      discrete_poles, discretize, is_stable)


class DiscretizationNode(Node):
    """把离散状态方程搬进 10~200 Hz 的定时器，观察采样周期的影响。"""

    def __init__(self):
        super().__init__('control_discretization')

        self.declare_parameter('method', 'tustin')
        self.declare_parameter('ts', 0.05)
        self.declare_parameter('wn', 3.0)
        self.declare_parameter('zeta', 0.4)
        self.declare_parameter('duration', 6.0)

        method = str(self.get_parameter('method').value)
        if method not in METHODS:
            raise ValueError(f'method 必须是 {METHODS} 之一，收到 {method}')
        self.method = method
        self.ts = float(self.get_parameter('ts').value)
        self.duration = float(self.get_parameter('duration').value)

        self.system = SecondOrderSystem(wn=float(self.get_parameter('wn').value),
                                        zeta=float(self.get_parameter('zeta').value))
        self.poles = discrete_poles(self.system, self.method, self.ts)
        self.stable = is_stable(self.system, self.method, self.ts)

        self.f, self.g = discretize(self.system, self.method, self.ts)

        self.state = [0.0, 0.0]
        self.steps = 0
        self.max_error = 0.0
        self.reference = self.system.sample_continuous(self.ts, self.duration)

        self.pub_y = self.create_publisher(Float64, '/control_discretization/y', 10)
        self.pub_metrics = self.create_publisher(
            String, '/control_discretization/metrics', 10)
        self.timer = self.create_timer(self.ts, self.on_timer)

        self.get_logger().info(
            f'离散化节点已启动：{METHOD_LABELS[self.method]}，Ts={self.ts:.3f}s，'
            f'wn={self.system.wn:.2f}，zeta={self.system.zeta:.2f}，'
            f'稳定={"是" if self.stable else "否"}')

    def on_timer(self):
        fx = [self.f[0][0] * self.state[0] + self.f[0][1] * self.state[1],
              self.f[1][0] * self.state[0] + self.f[1][1] * self.state[1]]
        self.state = [fx[0] + self.g[0][0] * 1.0,
                      fx[1] + self.g[1][0] * 1.0]
        self.steps += 1

        value = self.system.output(self.state)
        if value != value or abs(value) > 1e6:
            self.get_logger().error('输出发散，停止本轮（说明该 Ts 下方法失稳）')
            self.timer.cancel()
            return

        if self.steps <= len(self.reference):
            error = abs(value - self.reference[self.steps - 1][1])
            self.max_error = max(self.max_error, error)

        message = Float64()
        message.data = float(value)
        self.pub_y.publish(message)

        if self.steps % max(1, int(round(1.0 / self.ts))) == 0:
            self._publish_metrics(value)

        if self.steps * self.ts >= self.duration:
            self._publish_metrics(value)
            self.timer.cancel()

    def _publish_metrics(self, value):
        text = (f'{METHOD_LABELS[self.method]} Ts={self.ts:.3f}s '
                f't={self.steps * self.ts:.2f}s y={value:.4f} '
                f'max_err={self.max_error:.5f} '
                f'poles={self.poles[0].real:+.4f} pm {abs(self.poles[0].imag):.4f}j '
                f'stable={self.stable}')
        message = String()
        message.data = text
        self.pub_metrics.publish(message)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = DiscretizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
