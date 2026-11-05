import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np


class UnderwaterSonarMapping(Node):
    def __init__(self):
        super().__init__('underwater_sonar_mapping')

        self.x = np.linspace(0, 20, 30)
        self.y = np.linspace(0, 20, 30)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        
        self.Z = 10 + 1.5 * np.sin(self.X / 2) + 1.5 * np.cos(self.Y / 2)

        self.obstacles = [
            (5, 5, 3), (12, 15, 4), (15, 8, 3.5)
        ]

        plt.ion()
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title('Underwater Sonar Mapping with Obstacles')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Depth (m)')
        
        self.surf = self.ax.plot_surface(self.X, self.Y, self.Z, cmap='ocean', alpha=0.6, edgecolor='none')

        for ox, oy, osize in self.obstacles:
            self.ax.bar3d(ox, oy, 10, osize, osize, osize, color='red', alpha=0.9)

        sonar_origin = (10, 10, 0)
        for i in range(5):
            angle = i * (2 * np.pi / 5)
            end_x = 10 + 8 * np.cos(angle)
            end_y = 10 + 8 * np.sin(angle)
            self.ax.plot([sonar_origin[0], end_x], [sonar_origin[1], end_y], 
                         [sonar_origin[2], 10], 'cyan', linewidth=1, alpha=0.8)

        self.ax.scatter([10], [10], [0], color='yellow', s=200, marker='^')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterSonarMapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
