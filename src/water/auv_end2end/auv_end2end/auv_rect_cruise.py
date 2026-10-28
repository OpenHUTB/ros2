#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AuvRectCruise(Node):
    def __init__(self):
        super().__init__('auv_rect_cruise')

        # 速度发布器
        self.vel_pub = self.create_publisher(Twist, '/auv/cmd_vel', 10)

        # 巡航参数
        self.forward_speed = 2.5    # 前进/后退速度，默认1.0，数值越大越快
        self.side_speed = 2.0       # 横向平移速度，默认0.8
        self.forward_time = 10.0    # 直行持续时间
        self.side_time = 20.0       # 横向持续时间

        # 状态机：0-前进  1-向右平移  2-后退  3-向左平移
        self.state = 0
        # 保存完整ROS时间对象，保留纳秒级精度
        self.start_time = self.get_clock().now()

        # 50ms定时循环
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info('水下矩形巡航节点启动 | 坐标系：ENU | Gazebo版本：11.10.2')

    def timer_cb(self):
        current_time = self.get_clock().now()
        # 纳秒转秒，保留完整小数精度，避免整数秒丢失时间
        elapsed = (current_time - self.start_time).nanoseconds() / 1e9
        vel = Twist()

        # 四状态闭合矩形逻辑
        if self.state == 0:
            # 状态0：沿x正方向前进
            vel.linear.x = self.forward_speed
            if elapsed >= self.forward_time:
                self.state = 1
                self.start_time = current_time
                self.get_logger().info('切换状态：向右平移')

        elif self.state == 1:
            # 状态1：沿y负方向右移
            vel.linear.y = -self.side_speed
            if elapsed >= self.side_time:
                self.state = 2
                self.start_time = current_time
                self.get_logger().info('切换状态：后退')

        elif self.state == 2:
            # 状态2：沿x负方向后退
            vel.linear.x = -self.forward_speed
            if elapsed >= self.forward_time:
                self.state = 3
                self.start_time = current_time
                self.get_logger().info('切换状态：向左平移')

        elif self.state == 3:
            # 状态3：沿y正方向左移回正
            vel.linear.y = self.side_speed
            if elapsed >= self.side_time:
                self.state = 0
                self.start_time = current_time
                self.get_logger().info('切换状态：前进 | 完成一个矩形循环')

        # 发布速度指令
        self.vel_pub.publish(vel)

    def destroy_node(self):
        # 停止时发布零速度，确保AUV立刻停下
        stop_vel = Twist()
        self.vel_pub.publish(stop_vel)
        self.get_logger().info('巡航节点停止，发布零速度指令')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AuvRectCruise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

