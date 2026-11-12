"""仓储 AGV 多车交通管理与路径规划.

演示 A* 多车路径规划 + 冲突避让（Reactive Traffic Management）。
不冲突于仓库现有 ground 分类的 CARLA、差速小车、R2D2 模块——
本模块专注于**仓储网格地图 + 多车调度**，场景完全不同。
"""
import json
import heapq
import numpy as np
from pathlib import Path


class GridMap:
    """网格化仓储地图."""

    def __init__(self, width, height, obstacles=None):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=int)
        if obstacles:
            for ox, oy in obstacles:
                if 0 <= oy < height and 0 <= ox < width:
                    self.grid[oy][ox] = 1

    def is_free(self, x, y):
        if not (0 <= x < self.width and 0 <= y < self.height):
            return False
        return self.grid[y][x] == 0

    def neighbors(self, x, y):
        dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        return [(x + dx, y + dy) for dx, dy in dirs
                if self.is_free(x + dx, y + dy)]


def a_star(map_, start, goal):
    """A* 寻路."""
    sx, sy = start
    gx, gy = goal
    open_set = []
    heapq.heappush(open_set, (0, sx, sy))
    g_score = {(sx, sy): 0}
    parent = {}

    while open_set:
        f, x, y = heapq.heappop(open_set)
        if (x, y) == (gx, gy):
            path = [(x, y)]
            while (x, y) in parent:
                x, y = parent[(x, y)]
                path.append((x, y))
            return list(reversed(path))
        for nx, ny in map_.neighbors(x, y):
            tentative = g_score[(x, y)] + 1
            if tentative < g_score.get((nx, ny), float("inf")):
                parent[(nx, ny)] = (x, y)
                g_score[(nx, ny)] = tentative
                h = abs(nx - gx) + abs(ny - gy)
                heapq.heappush(open_set, (tentative + h, nx, ny))
    return None


class AGV:
    """仓储 AGV."""

    def __init__(self, agv_id, start, goal):
        self.id = agv_id
        self.position = start
        self.goal = goal
        self.path = []
        self.path_index = 0
        self.waiting = False
        self.battery = 100.0
        self.done = False

    def recompute_path(self, map_, blocked=None):
        if blocked is None:
            blocked = set()
        # 临时更新地图
        saved = map_.grid.copy()
        for bx, by in blocked:
            if 0 <= by < map_.height and 0 <= bx < map_.width:
                map_.grid[by][bx] = 2  # 临时占用
        path = a_star(map_, self.position, self.goal)
        map_.grid = saved
        if path:
            self.path = path
            self.path_index = 1
            self.waiting = False

    def step(self, occupied):
        """一步移动，返回 True 如果成功."""
        if self.done:
            return False
        if not self.path or self.path_index >= len(self.path):
            return False
        next_pos = self.path[self.path_index]
        if next_pos in occupied and next_pos != self.position:
            self.waiting = True
            return False
        self.position = next_pos
        self.path_index += 1
        self.battery -= 0.05
        self.waiting = False
        if self.position == self.goal:
            self.done = True
        return True


def generate_warehouse_map(width=20, height=15):
    """生成带货架通道的仓储地图."""
    obstacles = []
    # 水平货架区（偶数行留通道）
    for row in range(2, height - 2, 3):
        for col in range(3, width - 3):
            obstacles.append((col, row))
    return GridMap(width, height, obstacles)


def run_demo():
    """运行仓储 AGV 演示."""
    cfg_path = Path(__file__).parent / "scenario.json"
    if cfg_path.exists():
        with open(cfg_path) as f:
            cfg = json.load(f)
    else:
        cfg = {
            "n_agvs": 6,
            "map_width": 20,
            "map_height": 15,
            "max_steps": 300,
        }

    n_agvs = cfg["n_agvs"]
    W, H = cfg["map_width"], cfg["map_height"]
    max_steps = cfg.get("max_steps", 300)

    print("=" * 55)
    print(f"  仓储 AGV 多车交通管理")
    print(f"  AGV 数量: {n_agvs}  地图: {W}x{H} 网格")
    print("=" * 55)

    map_ = generate_warehouse_map(W, H)

    # 起点和终点（地图空闲区域）
    starts = [(1, 1), (1, 13), (3, 1), (3, 13), (5, 1), (5, 13)]
    goals = [(17, 13), (17, 1), (15, 13), (15, 1), (13, 13), (13, 1)]

    # 创建 AGV
    agvs = []
    for i in range(n_agvs):
        s = starts[i % len(starts)]
        g = goals[i % len(goals)]
        agv = AGV(i, s, g)
        agv.recompute_path(map_)
        agvs.append(agv)

    print("\n  📦 AGV 任务:")
    for a in agvs:
        print(f"     AGV-{a.id}: {a.position} → {a.goal}  (路径长度 {len(a.path)})")

    conflicts = 0
    stuck_count = 0
    for step in range(max_steps):
        occupied = set()
        for a in agvs:
            if not a.done:
                occupied.add(a.position)

        moved = True
        for a in agvs:
            if a.done:
                continue
            if a.waiting:
                conflicts += 1
            ok = a.step(occupied)
            if not ok and not a.done:
                stuck_count += 1
                # 冲突处理：重新规划
                blocked = set(occupied) - {a.position}
                a.recompute_path(map_, blocked)

        if step % 30 == 0:
            done_count = sum(1 for a in agvs if a.done)
            waiting = sum(1 for a in agvs if a.waiting)
            print(f"  Step {step:4d} | 完成: {done_count}/{n_agvs} | 等待: {waiting}")

        if all(a.done for a in agvs):
            print(f"  ✅ 所有 AGV 到达目标! 用时 {step} 步")
            break

    print(f"\n  📊 统计:")
    print(f"     冲突事件: {conflicts}")
    print(f"     重新规划次数: {stuck_count}")
    for a in agvs:
        status = "✅ 完成" if a.done else "⏳ 未完成"
        print(f"     AGV-{a.id}: {status}  剩余电量: {a.battery:.1f}%")


if __name__ == "__main__":
    run_demo()
