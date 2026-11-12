"""ROS 2 节点：订阅电机转速与轮速，用龙伯格观测器估计传动轴扭角。

话题
----
- 订阅 ``/drivetrain/motor_speed``（``std_msgs/Float64``，rad/s）
- 订阅 ``/drivetrain/wheel_speed``（``std_msgs/Float64``，rad/s）
- 发布 ``/wheel_speed_observer/theta_hat``（``std_msgs/Float64``，rad）
- 发布 ``/wheel_speed_observer/status``（``std_msgs/String``）

参数：``jw``、``k``、``c``、``zeta``、``bandwidth_factor``、``dt``。
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .observer import DrivetrainObserver


class WheelSpeedObserverNode(Node):
    """按固定步长推进观测器；两个输入都收到后才开始更新。"""

    def __init__(self):
        super().__init__('wheel_speed_observer')

        self.declare_parameter('jw', 0.6)
        self.declare_parameter('k', 1200.0)
        self.declare_parameter('c', 2.0)
        self.declare_parameter('zeta', 0.7)
        self.declare_parameter('bandwidth_factor', 1.0)
        self.declare_parameter('dt', 0.001)

        self.plant = DrivetrainObserver(
            jw=float(self.get_parameter('jw').value),
            k=float(self.get_parameter('k').value),
            c=float(self.get_parameter('c').value),
        )
        self.dt = float(self.get_parameter('dt').value)
        base_wn = math.sqrt(self.plant.k / self.plant.jw)
        wn = float(self.get_parameter('bandwidth_factor').value) * base_wn
        self.gain, self.target_poles = self.plant.design(
            wn=wn, zeta=float(self.get_parameter('zeta').value))
        self.observer_poles = self.plant.observer_poles(self.gain)

        self.estimate = [0.0, 0.0]
        self.motor_speed = None
        self.wheel_speed = None
        self.updates = 0

        self.sub_motor = self.create_subscription(
            Float64, '/drivetrain/motor_speed', self.on_motor, 20)
        self.sub_wheel = self.create_subscription(
            Float64, '/drivetrain/wheel_speed', self.on_wheel, 20)
        self.theta_pub = self.create_publisher(
            Float64, '/wheel_speed_observer/theta_hat', 10)
        self.status_pub = self.create_publisher(
            String, '/wheel_speed_observer/status', 10)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f'观测器节点启动：wn={wn:.1f} rad/s，zeta={self.target_poles[0].real:.1f}'
            f'±{abs(self.target_poles[0].imag):.1f}j')
        self.get_logger().info(
            f'增益 L = [{self.gain[0][0]:.3f}, {self.gain[1][0]:.3f}]，'
            f'观测器极点 {self.observer_poles[0]:.2f} / {self.observer_poles[1]:.2f}')
        self.get_logger().info(
            f'能观性 det = {self.plant.observability_determinant():.2f}')

    def on_motor(self, msg):
        self.motor_speed = float(msg.data)

    def on_wheel(self, msg):
        self.wheel_speed = float(msg.data)

    def on_timer(self):
        if self.motor_speed is None or self.wheel_speed is None:
            self.get_logger().warning(
                '等待 /drivetrain/motor_speed 与 /drivetrain/wheel_speed',
                throttle_duration_sec=2.0)
            return

        theta, omega = self.estimate
        measured = self.wheel_speed
        residual = measured - omega
        dtheta = self.motor_speed - omega + self.gain[0][0] * residual
        domega = ((self.plant.k * theta
                   + self.plant.c * (self.motor_speed - omega)) / self.plant.jw
                  + self.gain[1][0] * residual)
        self.estimate = [theta + dtheta * self.dt, omega + domega * self.dt]
        self.updates += 1

        message = Float64()
        message.data = float(self.estimate[0])
        self.theta_pub.publish(message)

        if self.updates % 500 == 0:
            status = String()
            status.data = (f'theta_hat={self.estimate[0]:.5f} rad '
                           f'omega_hat={self.estimate[1]:.3f} rad/s '
                           f'residual={residual:.5f} updates={self.updates}')
            self.status_pub.publish(status)
            self.get_logger().info(status.data)


def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedObserverNode()
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
