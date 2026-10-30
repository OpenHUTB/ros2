#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""headless_test.py - 无界面验证：规划路径 + 跟踪控制，看最终能否停到目标位姿。"""
import math
from collision_check import VehicleFootprint, check_collision
from hybrid_a_star import HybridAStar
from tracker import DiffRCSim, PurePursuit

CAR = VehicleFootprint(length=4.4, width=1.9, wheel_base=2.7)
SLOT_W, SLOT_D = 2.6, 5.2
LEFT_X0, GOAL_X0, RIGHT_X0 = -5.5, -2.9, -0.3
OBSTACLES = [
    (LEFT_X0 + SLOT_W/2, SLOT_D/2, CAR.width, CAR.length),
    (RIGHT_X0 + SLOT_W/2, SLOT_D/2, CAR.width, CAR.length),
]
START = (2.5, -2.2, 0.0)
GOAL = (GOAL_X0 + SLOT_W/2, SLOT_D/2, math.pi/2)

print("=== 1. Hybrid A* 规划 ===")
planner = HybridAStar(CAR, OBSTACLES, step=0.8, max_iter=80000)
path = planner.plan(START, GOAL, goal_tol_xy=0.5, goal_tol_yaw=math.radians(25))
if path is None:
    print("!! 规划失败")
    raise SystemExit(1)
# 路径加密：把相邻节点间线性插值到 ~0.1m 间距
from tracker import _densify  # noqa
path = _densify(path, 0.1)
print("路径点数:", len(path))
print("起点:", path[0])
print("终点:", path[-1])

# 检查路径全程无碰撞
blocked = [p for p in path if check_collision(p[0], p[1], p[2], CAR, OBSTACLES)]
print("路径碰撞点数:", len(blocked))

print("\n=== 2. Pure Pursuit 跟踪 ===")
car = DiffRCSim(CAR)
car.reset(*START)
ctrl = PurePursuit(path, CAR, lookahead=1.0, speed=0.7)
dt = 0.05
for step in range(2000):
    v, s, info = ctrl.run(car, dt)
    car.step(v, s, dt)
    if info.get("at_goal"):
        print("到达终点，步数:", step, "仿真时间: %.1f s" % (step*dt))
        break
else:
    print("!! 2000 步未到达")

print("\n=== 3. 最终位姿 ===")
print("目标:  x=%.2f  y=%.2f  yaw=%.0f°" % (GOAL[0], GOAL[1], math.degrees(GOAL[2])))
print("实际:  x=%.2f  y=%.2f  yaw=%.0f°" % (car.x, car.y, math.degrees(car.yaw)))
print("横向误差: %.2f m" % math.hypot(car.x-GOAL[0], car.y-GOAL[1]))
# 停车后是否还撞
print("停车后碰撞:", check_collision(car.x, car.y, car.yaw, CAR, OBSTACLES))
