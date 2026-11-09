"""把推力分配接到 ROS 2 话题: 订阅期望 wrench, 发布四桨推力与可行性状态."""
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .core import QuadrotorAllocator

# 参数名与默认值, 启动时读取一次; failed_rotor 取 -1 表示四桨完好.
PARAMETERS = (('mass', 2.0), ('arm_length', 0.25), ('torque_ratio', 0.016),
              ('max_thrust', 0.0), ('failed_rotor', -1))


class RotorAllocationNode(Node):
    """订阅机体期望 wrench, 按当前桨状态发布四桨推力."""

    def __init__(self):
        super().__init__('uav_rotor_failure_allocation')
        for name, value in PARAMETERS:
            self.declare_parameter(name, value)
        max_thrust = self.get_parameter('max_thrust').value or None
        self.allocator = QuadrotorAllocator(
            mass=self.get_parameter('mass').value,
            arm_length=self.get_parameter('arm_length').value,
            torque_ratio=self.get_parameter('torque_ratio').value,
            max_thrust=max_thrust)
        self.failed_rotor = int(self.get_parameter('failed_rotor').value)
        if self.failed_rotor >= 0 and self.failed_rotor not in (0, 1, 2, 3):
            raise ValueError('failed_rotor 必须为 -1 或 0..3')
        self.thrusts = self.create_publisher(Float64MultiArray, 'uav/rotor_thrust', 10)
        self.status = self.create_publisher(String, 'uav/allocation_status', 10)
        self.subscription = self.create_subscription(
            WrenchStamped, 'uav/desired_wrench', self.receive, 10)

    def receive(self, message):
        """只取 force.z 作为总推力, 水平力不参与分配."""
        request = [message.wrench.force.z, message.wrench.torque.x,
                   message.wrench.torque.y, message.wrench.torque.z]
        if self.failed_rotor >= 0:
            values, feasible, achieved = self.allocator.allocate_with_failure(
                request, self.failed_rotor)
            detail = 'failed=%d yaw=%.4f' % (self.failed_rotor, achieved[3])
        else:
            values, feasible = self.allocator.allocate(request)
            detail = 'healthy'
        self.thrusts.publish(Float64MultiArray(data=[float(v) for v in values]))
        self.status.publish(String(data=('ok' if feasible else 'infeasible') + ' ' + detail))
        if not feasible:
            self.get_logger().warning('期望 wrench 超出单桨推力上限: %s' % detail)


def main(args=None):
    """启动节点并阻塞在 spin 中."""
    rclpy.init(args=args)
    node = None
    try:
        node = RotorAllocationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
