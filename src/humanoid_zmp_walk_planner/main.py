import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import math

class HumanoidZMPPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_zmp_walk_planner')
        self.get_logger().info('人形机器人 ZMP 轨迹规划节点已启动。')

        self.step_length = 0.6
        self.step_height = 0.15
        self.step_time = 1.2
        self.t = 0.0

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.ax.set_xlim(-2, 8)
        self.ax.set_ylim(-0.5, 1.8)
        self.ax.set_title('Humanoid Robot ZMP Walking Trajectory')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(True, linestyle='--', alpha=0.7)
        
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        self.t += 0.05
        phase = (self.t % self.step_time) / self.step_time

        if phase < 0.5:
            x_body = phase * self.step_length * 2
            y_left_foot = 0.0
            y_right_foot = 0.0
            state = '左腿支撑 / 右腿摆动'
        else:
            x_body = (phase - 0.5) * self.step_length * 2 + self.step_length
            y_left_foot = 0.0
            y_right_foot = self.step_height * math.sin((phase - 0.5) * math.pi * 2)
            state = '右腿支撑 / 左腿摆动'

        self.ax.clear()
        self.ax.set_xlim(-2, 8)
        self.ax.set_ylim(-0.5, 1.8)
        self.ax.set_title('Humanoid Robot ZMP Walking Trajectory')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(True, linestyle='--', alpha=0.7)

        self.ax.add_patch(plt.Circle((x_body, 1.5), 0.12, color='blue'))
        self.ax.add_patch(plt.Rectangle((x_body - 0.15, 1.0), 0.3, 0.4, color='blue'))
        self.ax.plot([x_body - 0.1, x_body - 0.1], [1.0, y_left_foot], 'b-', linewidth=3)
        self.ax.plot([x_body + 0.1, x_body + 0.1], [1.0, y_right_foot], 'b-', linewidth=3)
        
        self.ax.text(0.5, 1.6, f'Status: {state}', fontsize=12, color='green')
        self.ax.legend()
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidZMPPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
