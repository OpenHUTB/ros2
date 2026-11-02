"""tracker_core 单元测试

重点回归：
1. 旧版 bug：目标在障碍物正后方时旧算法原地打转死锁——
   新算法必须在限定步数内绕过障碍物到达目标；
2. 多障碍/窄门/错位围墙等压力场景全部可达且不穿模；
3. 基础运动学与轨迹封顶。

运行：python -m pytest test_tracker_core.py -v
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tracker_core as tc

DT = 1.0 / 60.0


def drive(car, goal, max_steps=5000):
    """驱动小车直到到达目标，返回 (是否到达, 步数)"""
    for i in range(max_steps):
        if car.auto_step(DT, *goal):
            return True, i
    return False, max_steps


def min_clearance(car, obstacles):
    """轨迹上到所有障碍物表面的最小净空"""
    return min(
        min(math.hypot(px - ox, py - oy) - r for ox, oy, r in obstacles)
        for px, py in car.trail)


# ---------- 基础 ----------

def test_normalize_angle():
    assert abs(tc.normalize_angle(math.pi + 0.2) - (-math.pi + 0.2)) < 1e-9
    assert abs(tc.normalize_angle(-3 * math.pi - 0.1) - (math.pi - 0.1)) < 1e-9
    assert abs(tc.normalize_angle(0.3) - 0.3) < 1e-9


def test_arrive_static_goal_no_obstacles():
    car = tc.Car(100, 300, 0.0)
    ok, steps = drive(car, (800, 300))
    assert ok and steps < 600


def test_trail_capped():
    car = tc.Car(100, 300, 0.0, trail_limit=100)
    for _ in range(300):
        car.auto_step(DT, 800, 300)
    assert len(car.trail) <= 100


def test_manual_step_moves_and_collides():
    car = tc.Car(100, 300, 0.0)
    x0, y0 = car.x, car.y
    car.manual_step(DT, forward=1, turn=0)
    assert car.x > x0
    # 正对障碍物时手动前进应被挡住
    car2 = tc.Car(360, 300, 0.0, obstacles=[(450, 300, 60)])
    car2.manual_step(DT, forward=1, turn=0)
    assert math.hypot(car2.x - 450, car2.y - 300) >= 60 + car2.car_radius - 1e-6


# ---------- 旧版死锁 bug 回归 ----------

def test_goal_directly_behind_obstacle():
    """旧版 bug 场景：目标在障碍物正后方，3000 步内必须到达（旧版死锁）"""
    obstacles = [(450, 300, 60)]
    car = tc.Car(100, 300, 0.0, obstacles=obstacles)
    ok, steps = drive(car, (800, 300), max_steps=3000)
    assert ok, '目标在障碍物正后方时死锁（未到达）'
    assert steps < 1500
    assert min_clearance(car, obstacles) > 0   # 全程不穿模


def test_original_scene_three_obstacles():
    """原演示场景的三个障碍物，直线穿越可达"""
    obstacles = [(300, 200, 35), (600, 380, 45), (450, 450, 30)]
    car = tc.Car(100, 300, 0.0, obstacles=obstacles)
    ok, steps = drive(car, (800, 300))
    assert ok and steps < 1500
    assert min_clearance(car, obstacles) > 0


# ---------- 多障碍压力场景 ----------

def test_narrow_gate():
    """两障碍物间 120px 窄门必须能穿过"""
    obstacles = [(450, 180, 60), (450, 420, 60)]
    car = tc.Car(100, 300, 0.0, obstacles=obstacles)
    ok, steps = drive(car, (800, 300))
    assert ok and steps < 2000
    assert min_clearance(car, obstacles) > 0


def test_staggered_wall():
    """错位三连围墙（势场局部极小场景）必须能绕出"""
    obstacles = [(300, 300, 70), (500, 220, 70), (700, 340, 70)]
    car = tc.Car(100, 300, 0.0, obstacles=obstacles)
    ok, steps = drive(car, (800, 300), max_steps=3000)
    assert ok, '错位围墙场景未到达'
    assert steps < 2000
    assert min_clearance(car, obstacles) > 0


def test_goal_between_obstacles():
    """目标位于两障碍物之间必须可达"""
    obstacles = [(450, 300, 60), (380, 300, 60)]
    car = tc.Car(450, 550, 0.0, obstacles=obstacles)
    ok, steps = drive(car, (450, 50))
    assert ok and steps < 2000
    assert min_clearance(car, obstacles) > 0


def test_goal_near_obstacle_surface():
    """目标贴近障碍物表面（在斥力影响圈内）必须可达"""
    obstacles = [(300, 300, 70), (500, 220, 70)]
    car = tc.Car(100, 300, 0.0, obstacles=obstacles)
    ok, steps = drive(car, (700, 340))
    assert ok and steps < 2000
    assert min_clearance(car, obstacles) > 0


def test_start_close_to_obstacle():
    """起点紧贴障碍物表面也能正常出发"""
    obstacles = [(260, 300, 60)]
    car = tc.Car(340, 300, 0.0, obstacles=obstacles)   # 净空 10px
    ok, steps = drive(car, (800, 300))
    assert ok and steps < 2000
    assert min_clearance(car, obstacles) > 0


# ---------- 移动目标 ----------

def test_pursue_moving_target():
    """追踪随机游走目标：20 秒内应多次接近到 40px 内"""
    import random
    rng = random.Random(42)
    car = tc.Car(450, 300, 0.0,
                 obstacles=[(300, 200, 35), (600, 380, 45)])
    target = tc.MovingTarget(700, 150, rng=rng)
    close = 0
    for _ in range(1200):
        car.auto_step(DT, target.x, target.y)
        target.step(DT)
        if math.hypot(target.x - car.x, target.y - car.y) < 40:
            close += 1
    assert close > 60, '20 秒内未能有效接近移动目标'
