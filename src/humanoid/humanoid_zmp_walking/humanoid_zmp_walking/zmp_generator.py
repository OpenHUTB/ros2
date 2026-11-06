import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class ZmpGenerator(Node):
    def __init__(self):
        super().__init__('zmp_generator_node')
        self.get_logger().info('ZMP trajectory generator started')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'zmp_trajectory', 10)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        msg = Float64MultiArray()
        # 幅度减半、频率减半，让机器人站得更稳
        angle = math.sin(self.t * 0.5) * 0.2
        msg.data = [float(angle)]
        self.publisher_.publish(msg)
        self.t += self.dt

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
