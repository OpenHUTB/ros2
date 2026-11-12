"""ROS 2 节点：按高度与电池电压实时给出悬停油门解。

话题
----
- 订阅 ``/uav/battery_voltage``（``std_msgs/Float64``，V）：可选，缺省用参数里的电压
- 订阅 ``/uav/altitude``（``std_msgs/Float64``，m）：可选
- 发布 ``/uav_hover/throttle``（``std_msgs/Float64``）：悬停油门（0~1，>1 表示飞不起来）
- 发布 ``/uav_hover/status``（``std_msgs/String``）

参数：``mass``、``cells``、``cell_voltage``、``altitude``、``diameter``、``rotors``、``rate_hz``。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .rotor import Rotor


class HoverThrottleNode(Node):
    """把悬停求解做成在线服务：任一输入变化就重新算一次。"""

    def __init__(self):
        super().__init__('uav_hover_throttle')

        self.declare_parameter('mass', 1.6)
        self.declare_parameter('cells', 4)
        self.declare_parameter('cell_voltage', 4.2)
        self.declare_parameter('altitude', 0.0)
        self.declare_parameter('diameter', 0.254)
        self.declare_parameter('rotors', 4)
        self.declare_parameter('rate_hz', 1.0)

        self.mass = float(self.get_parameter('mass').value)
        self.rotors = int(self.get_parameter('rotors').value)
        self.rotor = Rotor(diameter_m=float(self.get_parameter('diameter').value))

        self.voltage = (float(self.get_parameter('cells').value)
                        * float(self.get_parameter('cell_voltage').value))
        self.altitude = float(self.get_parameter('altitude').value)
        self.dirty = True

        self.throttle_pub = self.create_publisher(Float64, '/uav_hover/throttle', 10)
        self.status_pub = self.create_publisher(String, '/uav_hover/status', 10)

        self.create_subscription(Float64, '/uav/battery_voltage',
                                self.on_voltage, 10)
        self.create_subscription(Float64, '/uav/altitude', self.on_altitude, 10)

        rate = float(self.get_parameter('rate_hz').value)
        if rate <= 0.0:
            raise ValueError('rate_hz 必须为正数')
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f'悬停油门节点启动：m={self.mass:.2f} kg，{self.rotors} 旋翼，'
            f'桨径 {self.rotor.diameter * 1000:.0f} mm，初始 {self.voltage:.1f} V')

    def on_voltage(self, msg):
        value = float(msg.data)
        if value != value or value <= 0.0:
            self.get_logger().warning('收到非法电压，已忽略')
            return
        self.voltage = value
        self.dirty = True

    def on_altitude(self, msg):
        value = float(msg.data)
        if value != value:
            self.get_logger().warning('收到 NaN 高度，已忽略')
            return
        self.altitude = value
        self.dirty = True

    def on_timer(self):
        if not self.dirty:
            return
        self.dirty = False

        report = self.rotor.hover_report(self.mass, self.voltage,
                                        self.altitude, self.rotors)
        message = Float64()
        message.data = float(report['throttle'])
        self.throttle_pub.publish(message)

        status = String()
        status.data = (f"alt={report['altitude_m']:.0f}m "
                       f"rho={report['density']:.4f} "
                       f"rpm={report['rpm_hover']:.0f}/{report['rpm_max']:.0f} "
                       f"throttle={report['throttle'] * 100:.1f}% "
                       f"power={report['power_w']:.1f}W "
                       f"feasible={report['feasible']}")
        self.status_pub.publish(status)
        self.get_logger().info(status.data)


def main(args=None):
    rclpy.init(args=args)
    node = HoverThrottleNode()
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
