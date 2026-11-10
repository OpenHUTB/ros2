#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class PidControlNode(Node):
    def __init__(self):
        super().__init__('pid_control_node')
        self.get_logger().info("PID 控制节点已启动！")
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.target_speed = 5.0
        self.current_speed = 0.0
        self.dt = 0.1
        self.subscription = self.create_subscription(Float32, '/car/current_speed', self.speed_callback, 10)
        self.publisher = self.create_publisher(Twist, '/car/cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def speed_callback(self, msg):
        self.current_speed = msg.data

    def timer_callback(self):
        control_output = self.pid.compute(self.target_speed, self.current_speed, self.dt)
        cmd = Twist()
        cmd.linear.x = control_output
        self.publisher.publish(cmd)
        self.get_logger().info(f"当前速度: {self.current_speed:.2f}, 目标: {self.target_speed}, 输出控制: {control_output:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PidControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
