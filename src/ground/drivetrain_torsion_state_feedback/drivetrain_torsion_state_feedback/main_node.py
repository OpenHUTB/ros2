"""ROS 2 节点：实时跑扭振自由响应，对比开环与状态反馈。

话题
----
- ``/drivetrain_torsion/theta``（``std_msgs/Float64``）：轴扭转角（rad）
- ``/drivetrain_torsion/torque``（``std_msgs/Float64``）：反馈转矩（Nm）
- ``/drivetrain_torsion/status``（``std_msgs/String``）：极点、阻尼比、增益

参数：``jm``、``jw``、``k``、``c``、``zeta``（闭环目标阻尼比）、``dt``、``initial_twist``。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .drivetrain import TwoInertiaDrivetrain


class TorsionNode(Node):
    """用 1 ms 定时器积分扭振方程，反馈项为 ``u = -K x``。"""

    def __init__(self):
        super().__init__('drivetrain_torsion_state_feedback')

        self.declare_parameter('jm', 0.05)
        self.declare_parameter('jw', 0.6)
        self.declare_parameter('k', 1200.0)
        self.declare_parameter('c', 2.0)
        self.declare_parameter('zeta', 0.7)
        self.declare_parameter('dt', 0.001)
        self.declare_parameter('initial_twist', 0.01)
        self.declare_parameter('feedback_enabled', True)

        self.plant = TwoInertiaDrivetrain(
            jm=float(self.get_parameter('jm').value),
            jw=float(self.get_parameter('jw').value),
            k=float(self.get_parameter('k').value),
            c=float(self.get_parameter('c').value),
        )
        zeta = float(self.get_parameter('zeta').value)
        self.dt = float(self.get_parameter('dt').value)
        self.feedback_enabled = bool(self.get_parameter('feedback_enabled').value)
        self.initial_twist = float(self.get_parameter('initial_twist').value)

        self.poles_target = self.plant.desired_poles(
            self.plant.natural_frequency, zeta)
        self.gain = self.plant.ackermann_gain(self.poles_target)
        _a_cl, poles, zeta_cl, wn_cl = self.plant.closed_loop(self.gain)

        self.state = [self.initial_twist, 0.0]

        self.theta_pub = self.create_publisher(Float64, '/drivetrain_torsion/theta', 10)
        self.torque_pub = self.create_publisher(Float64, '/drivetrain_torsion/torque', 10)
        self.status_pub = self.create_publisher(
            String, '/drivetrain_torsion/status', 10)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f'扭振节点启动：wn={self.plant.natural_frequency:.1f} rad/s，'
            f'开环 zeta={self.plant.damping_ratio:.3f}，'
            f'闭环 zeta={zeta_cl:.3f}（{wn_cl:.1f} rad/s）')
        self.get_logger().info(
            f'Ackermann 增益 K = [{self.gain[0][0]:.4f}, {self.gain[0][1]:.4f}]，'
            f'反馈{"开启" if self.feedback_enabled else "关闭（开环）"}')
        self._publish_status(zeta_cl, poles)

    def on_timer(self):
        x = self.state
        torque = 0.0
        if self.feedback_enabled:
            torque = -(self.gain[0][0] * x[0] + self.gain[0][1] * x[1])

        dtheta = x[1]
        domega = self.plant.s * (torque - self.plant.k * x[0] - self.plant.c * x[1])
        self.state = [x[0] + dtheta * self.dt, x[1] + domega * self.dt]

        theta_msg = Float64()
        theta_msg.data = float(self.state[0])
        self.theta_pub.publish(theta_msg)

        torque_msg = Float64()
        torque_msg.data = float(torque)
        self.torque_pub.publish(torque_msg)

    def _publish_status(self, zeta_cl, poles):
        message = String()
        message.data = (f'open_loop zeta={self.plant.damping_ratio:.3f} '
                        f'closed_loop zeta={zeta_cl:.3f} '
                        f'poles={poles[0]:.3f},{poles[1]:.3f} '
                        f'K=[{self.gain[0][0]:.4f},{self.gain[0][1]:.4f}] '
                        f'feedback={self.feedback_enabled}')
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TorsionNode()
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
