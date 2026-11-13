# -*- coding: utf-8 -*-
"""快速随机扩展树（Rapidly-exploring Random Tree, RRT）路径规划。

核心思想：从起点出发，每次在自由空间随机采样一个点，找到树中离它
最近的节点，朝该采样点扩展一个固定步长的新节点；若新节点连线不与
障碍相交则加入树。以小概率直接朝目标采样以加速收敛，当新节点落入
目标半径内即认为规划成功，再从目标节点沿 parent 链回溯出路径。

本模块为纯算法实现，不依赖 pygame，便于单元测试与复现。
"""
from __future__ import annotations
import math
import random
from typing import List, Optional, Tuple

from .map import ObstacleMap


class Node:
    """RRT 树节点。"""

    __slots__ = ("x", "y", "parent")

    def __init__(self, x: float, y: float, parent: Optional["Node"] = None):
        self.x = float(x)
        self.y = float(y)
        self.parent = parent

    def dist_to(self, px: float, py: float) -> float:
        return math.hypot(self.x - px, self.y - py)

    def __repr__(self) -> str:  # pragma: no cover - 调试用
        return f"Node({self.x:.1f},{self.y:.1f})"


class RRT:
    """RRT 路径规划器。

    Parameters
    ----------
    obs_map : ObstacleMap
        障碍物地图，提供线段碰撞检测。
    start : tuple(float, float)
        起点坐标。
    goal : tuple(float, float)
        目标点坐标。
    step_size : float
        单次扩展步长上限。
    goal_radius : float
        目标判定半径：新节点距目标小于该值即视为到达。
    goal_sample_rate : float
        每次采样时直接取目标点的概率，范围 [0, 1]。
    seed : int, optional
        随机种子，传入则结果可复现。
    """

    def __init__(self, obs_map: ObstacleMap, start: Tuple[float, float],
                 goal: Tuple[float, float], step_size: float,
                 goal_radius: float, goal_sample_rate: float,
                 seed: Optional[int] = None):
        self.map = obs_map
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.step_size = float(step_size)
        self.goal_radius = float(goal_radius)
        self.goal_sample_rate = float(goal_sample_rate)
        self.rng = random.Random(seed)
        self.tree: List[Node] = [self.start]
        self.node_reached_goal: Optional[Node] = None

    # ---- 采样 ----
    def sample(self) -> Tuple[float, float]:
        """在画布范围内随机采样一个点，以 goal_sample_rate 概率直接取目标。"""
        if self.rng.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        x = self.rng.uniform(0.0, self.map.width)
        y = self.rng.uniform(0.0, self.map.height)
        return (x, y)

    # ---- 最近邻 ----
    def nearest(self, tree: List[Node], px: float, py: float) -> Node:
        """返回树中离 (px, py) 最近的节点。树非空由调用方保证。"""
        best = tree[0]
        best_d = best.dist_to(px, py)
        for n in tree[1:]:
            d = n.dist_to(px, py)
            if d < best_d:
                best, best_d = n, d
        return best

    # ---- 扩展 ----
    def steer(self, from_node: Node, tx: float, ty: float) -> Node:
        """从 from_node 朝 (tx, ty) 方向扩展一个不超过 step_size 的新节点。

        新节点不进行碰撞检测（由 plan 负责），仅保证方向正确与步长受限。
        """
        dx = tx - from_node.x
        dy = ty - from_node.y
        dist = math.hypot(dx, dy)
        if dist <= self.step_size or dist == 0:
            new_x, new_y = tx, ty
        else:
            ratio = self.step_size / dist
            new_x = from_node.x + dx * ratio
            new_y = from_node.y + dy * ratio
        return Node(new_x, new_y, parent=from_node)

    # ---- 回溯 ----
    def retrace(self, node: Node) -> List[Node]:
        """从 node 沿 parent 链回溯到起点，返回起点到 node 的路径列表。"""
        path: List[Node] = []
        cur: Optional[Node] = node
        while cur is not None:
            path.append(cur)
            cur = cur.parent
        path.reverse()
        return path

    # ---- 主循环 ----
    def plan(self, max_iter: int) -> List[Node]:
        """执行 RRT 规划。

        Returns
        -------
        list[Node]
            成功时返回起点到目标的路径节点列表（含起终点）；
            失败时返回空列表。
        """
        self.tree = [self.start]
        self.node_reached_goal = None
        for _ in range(int(max_iter)):
            sx, sy = self.sample()
            near = self.nearest(self.tree, sx, sy)
            new_node = self.steer(near, sx, sy)
            # 碰撞检测：连线不能穿过障碍，新节点也不能落在障碍内
            if self.map.segment_collides(near.x, near.y, new_node.x, new_node.y):
                continue
            if self.map.point_in_obstacle(new_node.x, new_node.y):
                continue
            self.tree.append(new_node)
            # 到达判定
            if new_node.dist_to(self.goal.x, self.goal.y) <= self.goal_radius:
                self.node_reached_goal = new_node
                return self.retrace(new_node)
        return []

    # ---- 统计 ----
    def path_length(self, path: List[Node]) -> float:
        """计算路径折线总长度。"""
        if len(path) < 2:
            return 0.0
        return sum(path[i].dist_to(path[i + 1].x, path[i + 1].y)
                   for i in range(len(path) - 1))
