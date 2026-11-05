import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive')
        # 车辆参数
        self.wheel_radius = 0.2    # 车轮半径
        self.wheel_separation = 0.7 # 轮距

        # 订阅速度指令
        self.sub = self.create_subscription(
            Twist, '/auv/cmd_vel', self.cmd_cb, 10)

        # 发布左右轮速度
        self.left_pub = self.create_publisher(
            Float64, '/auv/left_wheel_joint/cmd_vel', 10)
        self.right_pub = self.create_publisher(
            Float64, '/auv/right_wheel_joint/cmd_vel', 10)

    def cmd_cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # 差速公式：计算左右轮角速度
        left_vel = (v - w * self.wheel_separation / 2) / self.wheel_radius
        right_vel = (v + w * self.wheel_separation / 2) / self.wheel_radius

        # 发布
        l_msg = Float64()
        r_msg = Float64()
        l_msg.data = left_vel
        r_msg.data = right_vel
        self.left_pub.publish(l_msg)
        self.right_pub.publish(r_msg)

def main():
    rclpy.init()
    node = DiffDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
