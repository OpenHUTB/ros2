import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

from humanoid_zmp_walk_planner.gait import leg_targets

class HumanoidZMPPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_zmp_walk_planner')
        self.get_logger().info('Humanoid ZMP trajectory planner node started.')

        self.step_length = 0.6
        self.step_height = 0.15
        self.step_time = 1.2
        self.t = 0.0

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        self.t += 0.05
        x_body, y_left_foot, y_right_foot, _phase, support = leg_targets(
            self.t, self.step_length, self.step_time, self.step_height)
        state = ('Left Leg Support / Right Leg Swing' if support == 'left'
                 else 'Right Leg Support / Left Leg Swing')

        self.ax.clear()
        # 动态视野跟随机器人
        self.ax.set_xlim(x_body - 3, x_body + 3) 
        self.ax.set_ylim(-0.5, 2.0)
        self.ax.set_title('Humanoid Robot Walking Animation (Simplified)')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(True, linestyle='--', alpha=0.7)

        self.ax.add_patch(plt.Circle((x_body, 1.5), 0.12, color='blue'))
        self.ax.add_patch(plt.Rectangle((x_body - 0.15, 1.0), 0.3, 0.4, color='blue'))
        
        self.ax.plot([x_body - 0.1, x_body - 0.1], [1.0, y_left_foot], 'b-', linewidth=3, label='Left Leg')
        self.ax.plot([x_body + 0.1, x_body + 0.1], [1.0, y_right_foot], 'b-', linewidth=3, label='Right Leg')
        
        self.ax.text(x_body - 2.0, 1.8, f'Status: {state}', fontsize=12, color='green')
        self.ax.legend(loc='upper right')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidZMPPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
