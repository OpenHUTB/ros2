import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 三维空间节点类
class RRT3DNode:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.parent = None
        self.cost = 0.0  # 从起点到当前节点代价

# RRT* 三维规划器（面向对象封装）
class RRTStar3DPlanner:
    def __init__(self, start, goal, obstacle_list, space_bounds, step=0.5, max_iter=800, goal_radius=0.6):
        self.start = RRT3DNode(start)
        self.goal = RRT3DNode(goal)
        self.obstacles = obstacle_list
        self.space_min = np.array([space_bounds[0], space_bounds[2], space_bounds[4]])
        self.space_max = np.array([space_bounds[1], space_bounds[3], space_bounds[5]])
        self.step = step          # 扩展步长
        self.max_iter = max_iter  # 最大迭代次数
        self.goal_radius = goal_radius
        self.nodes = [self.start] # 树节点集合

    # 随机采样三维空间点
    def random_sample(self):
        x = random.uniform(self.space_min[0], self.space_max[0])
        y = random.uniform(self.space_min[1], self.space_max[1])
        z = random.uniform(self.space_min[2], self.space_max[2])
        return np.array([x, y, z])

    # 找树中距离采样点最近的节点
    def nearest_node(self, sample_pt):
        nearest = self.nodes[0]
        min_dist = np.linalg.norm(sample_pt - nearest.pos)
        for node in self.nodes:
            dist = np.linalg.norm(sample_pt - node.pos)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest

    # 从父节点往采样点延伸，得到新节点
    def steer(self, near_node, sample_pt):
        vec = sample_pt - near_node.pos
        dist = np.linalg.norm(vec)
        if dist < self.step:
            new_pos = sample_pt
        else:
            vec_unit = vec / dist
            new_pos = near_node.pos + vec_unit * self.step
        return RRT3DNode(new_pos)

    # 碰撞检测：线段是否穿过障碍物（球体障碍物）
    def collision_check(self, p1, p2):
        for obs in self.obstacles:
            obs_pos, obs_r = obs
            d = self.point_segment_distance(obs_pos, p1, p2)
            if d < obs_r:
                return True
        return False

    # 点到线段距离
    def point_segment_distance(self, point, seg_start, seg_end):
        seg_vec = seg_end - seg_start
        point_vec = point - seg_start
        t = np.clip(np.dot(point_vec, seg_vec)/np.dot(seg_vec, seg_vec), 0, 1)
        proj = seg_start + t * seg_vec
        return np.linalg.norm(point - proj)

    # 寻找新节点附近一定范围内的节点（RRT*重选父节点）
    def find_near_nodes(self, new_node, radius=1.2):
        near_nodes = []
        for node in self.nodes:
            if np.linalg.norm(node.pos - new_node.pos) < radius:
                near_nodes.append(node)
        return near_nodes

    # 主规划函数
    def plan(self):
        for i in range(self.max_iter):
            # 1.随机采样
            sample_p = self.random_sample()
            # 2.找最近节点
            nearest = self.nearest_node(sample_p)
            # 3.生长出新节点
            new_node = self.steer(nearest, sample_p)
            # 4.碰撞检测
            if self.collision_check(nearest.pos, new_node.pos):
                continue

            # RRT*：重选父节点
            near_nodes = self.find_near_nodes(new_node)
            min_cost = nearest.cost + np.linalg.norm(new_node.pos - nearest.pos)
            best_parent = nearest
            for n in near_nodes:
                if not self.collision_check(n.pos, new_node.pos):
                    cost_candidate = n.cost + np.linalg.norm(new_node.pos - n.pos)
                    if cost_candidate < min_cost:
                        min_cost = cost_candidate
                        best_parent = n
            new_node.parent = best_parent
            new_node.cost = min_cost
            self.nodes.append(new_node)

            # RRT*：重新布线
            for n in near_nodes:
                if not self.collision_check(new_node.pos, n.pos):
                    if new_node.cost + np.linalg.norm(new_node.pos - n.pos) < n.cost:
                        n.parent = new_node
                        n.cost = new_node.cost + np.linalg.norm(new_node.pos - n.pos)

            # 判断是否到达目标区域
            if np.linalg.norm(new_node.pos - self.goal.pos) < self.goal_radius:
                self.goal.parent = new_node
                self.goal.cost = new_node.cost
                self.nodes.append(self.goal)
                print(f"✅ 寻路成功，迭代次数：{i}")
                # 回溯提取路径
                path = []
                cur = self.goal
                while cur is not None:
                    path.append(cur.pos)
                    cur = cur.parent
                return np.array(path[::-1])
        print("❌ 达到最大迭代次数，未找到路径")
        return None


if __name__ == "__main__":
	    # ========== 主程序入口 ==========
	    # 起点、终点 三维坐标（无人机）
	    start_point = [0.5, 0.5, 0.5]
	    goal_point = [9.5, 9.5, 9.5]

	    # 障碍物列表：(中心x,y,z,球体半径)
	    obstacle_list = [
	        ([3, 3, 3], 0.8),
	        ([5, 4, 6], 0.7),
	        ([7, 7, 4], 0.9)
	    ]
	    # 三维空间范围 [xmin,xmax, ymin,ymax, zmin,zmax]
	    space = [0, 10, 0, 10, 0, 10]

	    planner = RRTStar3DPlanner(start_point, goal_point, obstacle_list, space, step=0.7, max_iter=3000)
	    path_result = planner.plan()

if path_result is not None:
    print("无人机三维路径点：")
    print(path_result)
    # ========== 3D可视化绘图 ==========
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 提取路径xyz坐标
    xs = [node[0] for node in path_result]
    ys = [node[1] for node in path_result]
    zs = [node[2] for node in path_result]

    # 绘制规划路径
    ax.plot(xs, ys, zs, 'r-o', linewidth=2, markersize=4, label="RRT* Path")

    # 起点绿色，终点蓝色标记
    ax.scatter(xs[0], ys[0], zs[0], c='green', s=100, label="Start")
    ax.scatter(xs[-1], ys[-1], zs[-1], c='blue', s=100, label="Goal")

     # 绘制球形障碍物
    for obs in obstacle_list:
        center, r = obs
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = center[0] + r * np.outer(np.cos(u), np.sin(v))
        y = center[1] + r * np.outer(np.sin(u), np.sin(v))
        z = center[2] + r * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color='gray', alpha=0.4)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title("3D RRT* UAV Path Planning")
        ax.legend()
        plt.savefig("rrt3d_result.png", dpi=300)
        print("图片已保存为 rrt3d_result.png")
    else:
        print("规划失败，可增大max_iter迭代次数重试")
