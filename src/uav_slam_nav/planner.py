#!/usr/bin/env python3
"""A* 路径规划：在占据栅格地图上搜索无碰撞路径"""
import numpy as np
import heapq
import os
import cv2

GRID_RES = 0.25
MAP_RANGE = 25.0
START = (0.0, 0.0)      # 起点（起飞点）
GOAL = (12.0, 12.0)     # 终点

grid = np.load(os.path.expanduser('~/ros2/src/uav_vision_nn/map.npy'))
N = grid.shape[0]

def w2g(x, y):
    return int((x + MAP_RANGE)/GRID_RES), int((y + MAP_RANGE)/GRID_RES)

def g2w(gx, gy):
    return (gx - MAP_RANGE/GRID_RES)*GRID_RES, (gy - MAP_RANGE/GRID_RES)*GRID_RES

start = w2g(*START)
goal = w2g(*GOAL)

def heuristic(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5

def astar(grid, start, goal):
    open_h = [(0, start)]
    g = {start: 0}
    parent = {}
    closed = set()
    while open_h:
        f, cur = heapq.heappop(open_h)
        if cur == goal:
            path = []
            while cur in parent:
                path.append(cur)
                cur = parent[cur]
            path.append(start)
            return path[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        for d in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            nb = (cur[0]+d[0], cur[1]+d[1])
            if 0 <= nb[0] < N and 0 <= nb[1] < N:
                if grid[nb[1], nb[0]] == 2:
                    continue
                if nb in closed:
                    continue
                ng = g[cur] + (1.414 if d[0] and d[1] else 1)
                if nb not in g or ng < g[nb]:
                    g[nb] = ng
                    parent[nb] = cur
                    heapq.heappush(open_h, (ng + heuristic(nb, goal), nb))
    return None

path = astar(grid, start, goal)
if path is None:
    print("未找到路径！")
    exit(1)
print(f"找到路径：{len(path)} 个栅格点")

# 保存路径
path_w = [g2w(gx, gy) for gx, gy in path]
np.save(os.path.expanduser('~/ros2/src/uav_vision_nn/path.npy'), np.array(path_w))

# 可视化
img = np.zeros((N, N, 3), dtype=np.uint8)
img[grid == 1] = [200, 200, 200]
img[grid == 2] = [0, 0, 255]
for gx, gy in path:
    img[gy, gx] = [0, 255, 0]
img[start[1], start[0]] = [255, 255, 0]
img[goal[1], goal[0]] = [255, 0, 255]
cv2.imwrite(os.path.expanduser('~/ros2/src/uav_vision_nn/path.png'), img)
print(f"路径图已保存: path.png (绿=路径, 黄=起点, 紫=终点)")
print(f"世界坐标航点: {path_w[:3]} ... {path_w[-1]}")
