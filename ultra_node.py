import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import random

class UltraAvoidNode(Node):
    def __init__(self):
        super().__init__('ultra_avoid_node')
        # 速度发布器，发布小车运动速度
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 创建定时器，0.5s执行一次，模拟超声波传感器
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.simulate_ultra)
        self.twist = Twist()

    def simulate_ultra(self):
        # 随机模拟障碍物距离，范围0.2~1.2米
        distance = random.uniform(0.2,1.2)
        # 避障判断逻辑
        if distance < 0.5:
            self.twist.linear.x = 0.0
            self.get_logger().info(f"⚠️检测障碍物！距离：{distance:.2f} m，小车停止")
        else:
            self.twist.linear.x = 0.2
            self.get_logger().info(f"✅无障碍物，距离：{distance:.2f} m，小车直行")
        self.pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = UltraAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
