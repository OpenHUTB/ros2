import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .gait import GaitPlan


class ZmpGenerator(Node):
    """发布 ZMP 参考轨迹与关节指令。

    话题 zmp_trajectory（Float64MultiArray，6 个元素）：
        [0] 时间戳
        [1] ZMP x
        [2] ZMP y
        [3] 质心 x
        [4] 质心 y
        [5] ZMP 是否位于支撑多边形内（1 是 / 0 否）

    话题 joint_commands（Float64MultiArray，6 个元素）：
        左髋、左膝、左踝、右髋、右膝、右踝的目标角（rad）
    """

    def __init__(self):
        super().__init__('zmp_generator_node')
        self.declare_parameter('com_height', 0.65)
        self.com_height = self.get_parameter('com_height').value
        self.plan = GaitPlan(com_height=self.com_height)
        self.get_logger().info(
            'ZMP 步态规划已生成：{} 步，规划时长 {:.2f} s'.format(
                len(self.plan.steps), self.plan.duration))
        inside = all(self.plan.in_support)
        self.get_logger().info(
            'ZMP 支撑域判定：{}'.format('全部在支撑多边形内' if inside
                                        else '存在越界采样点'))

        self.zmp_pub = self.create_publisher(
            Float64MultiArray, 'zmp_trajectory', 10)
        self.joint_pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10)
        self.dt = self.plan.dt
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        elapsed = (self.get_clock().now() -
                   self.start_time).nanoseconds * 1e-9
        t = elapsed % self.plan.duration
        index = min(int(t / self.plan.dt), len(self.plan.t) - 1)

        zmp_x, zmp_y = self.plan.zmp[index]
        com_x, com_y = self.plan.com[index]
        message = Float64MultiArray()
        message.data = [float(self.plan.t[index]),
                        float(zmp_x), float(zmp_y),
                        float(com_x), float(com_y),
                        float(1.0 if self.plan.in_support[index] else 0.0)]
        self.zmp_pub.publish(message)

        command = Float64MultiArray()
        command.data = [float(angle)
                        for angle in self.plan.joint_targets(t)]
        self.joint_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ZmpGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
