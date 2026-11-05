import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.state = 0
        self.get_logger().info('轨迹发布节点已启动。')

    def timer_callback(self):
        msg = Twist()
        self.state += 1
        if self.state % 3 == 1:
            msg.linear.x = 1.0
            self.get_logger().info('发布: 前进指令')
        elif self.state % 3 == 2:
            msg.angular.z = 1.0
            self.get_logger().info('发布: 旋转指令')
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('发布: 悬停指令')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
