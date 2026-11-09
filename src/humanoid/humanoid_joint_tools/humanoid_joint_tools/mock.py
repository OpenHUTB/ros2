import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MockDriver(Node):
    def __init__(self):
        super().__init__('mock_driver')
        self.declare_parameter('topic', '/joint_states_raw')
        self.declare_parameter('rate_hz', 20.0)
        topic = self.get_parameter('topic').value
        rate = float(self.get_parameter('rate_hz').value)
        self.period = 1.0 / rate
        self.publisher_ = self.create_publisher(JointState, topic, 20)
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.t = 0.0
        self.get_logger().info(f'Publishing mock joint states on {topic} at {rate} Hz')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_shoulder_joint', 'right_shoulder_joint']
        msg.position = [math.sin(self.t), math.sin(self.t + math.pi)]
        self.t += self.period
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
