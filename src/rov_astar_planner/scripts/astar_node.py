#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

class AStarNode(Node):
    def __init__(self):
        super().__init__('astar_node')
        
        # 定义 10x10 的网格地图 (0为可通行，1为障碍物)
        self.grid = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 1, 1, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]
        
        self.start = (0, 0) # 起点
        self.goal = (9, 9)  # 终点
        
        # 延迟 1 秒执行，让节点启动完成
        self.timer = self.create_timer(1.0, self.run_astar)
        self.executed = False
        self.get_logger().info('水下 A* 路径规划节点已启动...')

    def get_neighbors(self, node):
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)] # 8个方向
        for dx, dy in directions:
            x, y = node[0] + dx, node[1] + dy
            if 0 <= x < 10 and 0 <= y < 10 and self.grid[x][y] == 0:
                neighbors.append((x, y))
        return neighbors

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def run_astar(self):
        if self.executed: return
        self.executed = True
        
        self.get_logger().info('开始计算 A* 最优路径...')
        open_set = {self.start}
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start, self.goal)}

        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                path.reverse()
                
                self.get_logger().info('✅ 路径规划成功！规划路径如下:')
                for p in path:
                    self.get_logger().info(f'  -> 航点: x={p[0]}, y={p[1]}')
                return

            open_set.remove(current)
            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, self.goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        
        self.get_logger().error('❌ 无法找到通往目标的路径！')

def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()