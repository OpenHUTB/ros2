import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class AuvRectCruise(Node):
    def __init__(self):
        super().__init__('auv_rect_cruise')
        
        # 速度发布器，复用现有仿真话题
        self.vel_pub = self.create_publisher(Twist, '/auv/cmd_vel', 10)
        
        
         # 巡航参数
        self.forward_speed = 10.0    # 直行速度
        self.side_speed = 10.0       # 横向平移速度
        self.forward_time = 10.0    # 直行持续时间
        self.side_time = 10.0       # 横向平移持续时间

        
        # 状态机：0-第一段直行  1-向右平移  2-第二段直行  3-向左平移回正
        self.state = 0
        self.start_time = time.time()
        
        # 50ms定时循环执行
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info('水下矩形巡航节点启动')

    def timer_cb(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        vel = Twist()

        # 状态切换逻辑
        if self.state == 0 or self.state == 2:
            # 直行状态
            vel.linear.x = self.forward_speed
            if elapsed > self.forward_time:
                self.state = (self.state + 1) % 4
                self.start_time = current_time
                self.get_logger().info('切换状态：横向平移')
        
        elif self.state == 1:
            # 向右平移
            vel.linear.y = -self.side_speed
            if elapsed > self.side_time:
                self.state = (self.state + 1) % 4
                self.start_time = current_time
                self.get_logger().info('切换状态：直行')
        
        elif self.state == 3:
            # 向左平移回正
            vel.linear.y = self.side_speed
            if elapsed > self.side_time:
                self.state = (self.state + 1) % 4
                self.start_time = current_time
                self.get_logger().info('切换状态：直行')

        self.vel_pub.publish(vel)


def main():
    rclpy.init()
    node = AuvRectCruise()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

