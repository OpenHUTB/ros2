import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class MockDriver(Node):
    def __init__(self):
        super().__init__('mock_driver')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_shoulder_joint', 'right_shoulder_joint']
        msg.position = [math.sin(self.t), math.sin(self.t + 3.14)]
        self.t += 0.05
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
