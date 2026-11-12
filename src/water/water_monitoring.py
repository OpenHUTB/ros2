"""AUV 水质监测自主采样任务规划.

根据目标区域内的水质采样点优先级，
生成覆盖巡游路径（Coverage Path Planning），
并模拟多 AUV 协同完成采样。
"""
import json
import math
import numpy as np
from pathlib import Path


class SamplingPoint:
    """采样点：位置 + 水质指标 + 优先级."""

    def __init__(self, idx, position, ph=7.0, do=8.0, turbidity=1.0, priority=1.0):
        self.idx = idx
        self.position = np.array(position, dtype=float)
        self.ph = ph                 # 酸碱度
        self.do = do                 # 溶解氧 mg/L
        self.turbidity = turbidity   # 浊度 NTU
        self.priority = priority     # 采样优先级 0~1
        self.sampled = False


class MissionPlanner:
    """任务规划器 —— 基于 TSP + 优先级调度."""

    def __init__(self, n_auvs=3):
        self.n_auvs = n_auvs

    def nearest_neighbor_tour(self, start, points):
        """贪心最近邻生成单 AUV 巡游路径."""
        remaining = points[:]
        tour = [start]
        current = start
        while remaining:
            # 考虑距离和优先级
            def score(p):
                dist = np.linalg.norm(p.position - current)
                return dist - p.priority * 2.0  # 高优先级优先

            remaining.sort(key=score)
            next_p = remaining.pop(0)
            tour.append(next_p.position)
            current = next_p.position
        return tour

    def split_points_among_auvs(self, points):
        """按优先级分组 + 空间聚类分配给多 AUV."""
        # 简单策略：按优先级切分，优先级前 1/3 归 AUV 0，以此类推
        sorted_points = sorted(points, key=lambda p: -p.priority)
        chunks = [[] for _ in range(self.n_auvs)]
        for i, p in enumerate(sorted_points):
            chunks[i % self.n_auvs].append(p)
        return chunks

    def generate_paths(self, points, auv_start_positions):
        """为每个 AUV 生成巡游路径."""
        chunks = self.split_points_among_auvs(points)
        paths = []
        for i, chunk in enumerate(chunks):
            if chunk:
                tour = self.nearest_neighbor_tour(
                    auv_start_positions[i], chunk
                )
                paths.append(tour)
            else:
                paths.append([auv_start_positions[i]])
        return paths


class AUVAgent:
    """执行采样的 AUV."""

    def __init__(self, auv_id, start_pos, speed=1.0):
        self.id = auv_id
        self.position = np.array(start_pos, dtype=float)
        self.speed = speed
        self.current_target = None
        self.trajectory = [np.array(start_pos)]
        self.sampled_count = 0

    def move_towards(self, target, dt=0.1):
        direction = target - self.position
        dist = np.linalg.norm(direction)
        if dist < 0.3:
            self.position = target.copy()
            self.trajectory.append(self.position.copy())
            return True
        step = self.speed * dt
        self.position += direction / dist * min(step, dist)
        self.trajectory.append(self.position.copy())
        return False


def generate_sampling_area(width=50, height=30, spacing=8):
    """在水域内生成规则网格采样点 + 随机扰动模拟实测数据."""
    points = []
    idx = 0
    for x in range(-width // 2, width // 2 + 1, spacing):
        for y in range(-height // 2, height // 2 + 1, spacing):
            # 加随机扰动
            pos = [x + np.random.uniform(-1, 1), y + np.random.uniform(-1, 1), -5.0]
            # 模拟水质（某些区域异常）
            if abs(x) < 10 and abs(y) < 5:
                # 中心区域：疑似污染，优先级高
                ph = np.random.uniform(5.5, 7.0)
                do = np.random.uniform(2.0, 5.0)
                turbidity = np.random.uniform(15.0, 30.0)
                priority = 0.9
            else:
                ph = np.random.uniform(6.5, 8.5)
                do = np.random.uniform(6.0, 10.0)
                turbidity = np.random.uniform(0.5, 5.0)
                priority = np.random.uniform(0.2, 0.6)
            points.append(SamplingPoint(idx, pos, ph, do, turbidity, priority))
            idx += 1
    return points


def run_demo():
    """运行水质监测演示."""
    cfg_path = Path(__file__).parent / "scenario.json"
    if cfg_path.exists():
        with open(cfg_path) as f:
            cfg = json.load(f)
    else:
        cfg = {
            "n_auvs": 3,
            "area_width": 50,
            "area_height": 30,
            "max_steps": 800,
        }

    n_auvs = cfg["n_auvs"]
    max_steps = cfg.get("max_steps", 800)

    print("=" * 55)
    print(f"  AUV 水质监测自主采样任务")
    print(f"  AUV 数量: {n_auvs}  水域: {cfg['area_width']}x{cfg['area_height']}m")
    print("=" * 55)

    # 生成采样区域
    points = generate_sampling_area(cfg["area_width"], cfg["area_height"])
    print(f"  生成采样点: {len(points)} 个")

    # AUV 起点
    start_positions = [
        np.array([-20.0 + i * 10.0, -12.0, -2.0]) for i in range(n_auvs)
    ]

    # 任务规划
    planner = MissionPlanner(n_auvs)
    paths = planner.generate_paths(points, start_positions)

    # 创建 AUV
    auvs = [AUVAgent(i, start_positions[i]) for i in range(n_auvs)]

    # 路径索引
    path_indices = [1 for _ in range(n_auvs)]

    # 按采样点归属排序
    point_assignments = {auv.id: [] for auv in auvs}
    for chunk_idx, chunk in enumerate(planner.split_points_among_auvs(points)):
        point_assignments[chunk_idx] = chunk

    print(f"\n  📋 任务分配:")
    for aid, assigned in point_assignments.items():
        high_priority = sum(1 for p in assigned if p.priority > 0.7)
        print(f"     AUV-{aid}: {len(assigned)} 点 (其中 {high_priority} 个高优)")

    # 主循环
    sampled_points = set()
    for step in range(max_steps):
        for i, auv in enumerate(auvs):
            pi = path_indices[i]
            path = paths[i]
            if pi >= len(path):
                continue
            target = path[pi]
            reached = auv.move_towards(target)
            if reached:
                path_indices[i] += 1
                # 检查附近是否有采样点
                for p in point_assignments[auv.id]:
                    if p.idx in sampled_points:
                        continue
                    if np.linalg.norm(p.position - auv.position) < 0.5:
                        p.sampled = True
                        sampled_points.add(p.idx)
                        auv.sampled_count += 1

        total_sampled = len(sampled_points)
        if step % 100 == 0:
            print(f"  Step {step:4d} | 已采样: {total_sampled}/{len(points)} | 完成率: {100*total_sampled/len(points):.1f}%")

        if total_sampled >= len(points):
            print(f"  ✅ 所有采样点已访问!")
            break

    print("\n  🏁 任务结束:")
    for auv in auvs:
        print(f"     AUV-{auv.id}: 采样 {auv.sampled_count} 点, 轨迹 {len(auv.trajectory)} 步")


if __name__ == "__main__":
    run_demo()
