import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import math

class TrajectoryAnalyzer(Node):
    def __init__(self):
        super().__init__('trajectory_analyzer')
        self.subscription = self.create_subscription(Twist, '/drone/cmd_vel', self.listener_callback, 10)
        self.get_logger().info('轨迹识别节点已启动，等待飞行数据...')
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-', label='Drone Path')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_title('UAV Flight Trajectory')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend()
        self.ax.grid(True)
        self.xs, self.ys = [], []

    def listener_callback(self, msg):
        if msg.linear.x > 0:
            self.get_logger().info('【轨迹识别】当前飞行状态：直线前进')
            self.x += 0.5 * math.cos(self.theta)
            self.y += 0.5 * math.sin(self.theta)
        elif msg.angular.z > 0:
            self.get_logger().info('【轨迹识别】当前状态：原地旋转')
            self.theta += 0.2
        else:
            self.get_logger().info('【轨迹识别】当前状态：悬停等待')

        self.xs.append(self.x)
        self.ys.append(self.y)
        self.line.set_data(self.xs, self.ys)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
