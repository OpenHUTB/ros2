#!/usr/bin/env python3
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AuvRectCruise(Node):
    def __init__(self):
        super().__init__('auv_rect_cruise')

        # 速度发布器
        self.vel_pub = self.create_publisher(Twist, '/auv/cmd_vel', 10)

        # 巡航参数：全部通过 ROS 参数暴露，可在 launch 或命令行覆盖
        self.declare_parameter('forward_speed', 2.5)  # 前进/后退速度
        self.declare_parameter('side_speed', 2.0)     # 横向平移速度
        self.declare_parameter('forward_time', 10.0)  # 直行/后退持续时间
        self.declare_parameter('side_time', 20.0)     # 横向平移持续时间

        self.forward_speed = self.get_parameter('forward_speed').value
        self.side_speed = self.get_parameter('side_speed').value
        self.forward_time = self.get_parameter('forward_time').value
        self.side_time = self.get_parameter('side_time').value

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
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
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

    def publish_stop(self):
        # 停止时发布零速度，确保AUV立刻停下
        self.vel_pub.publish(Twist())
        self.get_logger().info('巡航节点停止，发布零速度指令')


def main(args=None):
    rclpy.init(args=args)
    node = AuvRectCruise()

    def handle_sigint(signum, frame):
        # rclpy 默认的 SIGINT 处理会先关闭 context，之后 publish 会失败，
        # 这里只抛出异常，把发零速度的时机留到 finally
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_sigint)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 若 context 已被外部关闭（如 launch 统一退出），此时 publish 会失败，
        # 但仍要保证节点正常销毁
        try:
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
