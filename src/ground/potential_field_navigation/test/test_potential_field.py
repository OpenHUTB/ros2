import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import math

from potential_field_navigation import config, field
from potential_field_navigation.robot import Robot


def test_attractive_points_to_goal():
    fx, fy = field.attractive_force((0.0, 0.0), (100.0, 0.0))
    assert abs(fx - 1.0) < 1e-9
    assert abs(fy) < 1e-9


def test_repulsion_zero_beyond_influence_radius():
    ob = (0.0, 0.0, 20.0)
    fx, fy = field.obstacle_repulsion((0.0, 120.0), ob)
    assert fx == 0.0 and fy == 0.0


def test_repulsion_grows_as_obstacle_gets_closer():
    ob = (0.0, 0.0, 20.0)
    near = field.obstacle_repulsion((0.0, 40.0), ob)
    far = field.obstacle_repulsion((0.0, 60.0), ob)
    assert math.hypot(*near) > math.hypot(*far) > 0.0


def test_repulsion_direction_away_from_obstacle():
    ob = (0.0, 0.0, 20.0)
    fx, fy = field.obstacle_repulsion((0.0, 40.0), ob)
    assert fy > 0.0 and abs(fx) < 1e-9


def test_robot_reaches_goal_around_obstacles():
    robot = Robot()
    steps = 0
    while not robot.done and steps < 3000:
        robot.step()
        steps += 1
    assert robot.done is True
    assert steps < 3000
    assert robot.distance() <= config.GOAL_TOL


def test_robot_keeps_clearance():
    robot = Robot()
    steps = 0
    while not robot.done and steps < 3000:
        robot.step()
        steps += 1
    assert robot.min_clearance > 0.0


def test_symmetric_obstacle_escapes_local_minimum():
    """障碍正好挡在起点与目标之间，纯势场会停在平衡点，靠切向偏置绕出。"""
    robot = Robot(start=(100.0, 300.0), goal=(700.0, 300.0),
                  obstacles=[(400.0, 300.0, 100.0)])
    steps = 0
    while not robot.done and steps < 4000:
        robot.step()
        steps += 1
    assert robot.done is True
    assert steps < 4000
    assert robot.min_clearance > 0.0


def test_total_force_accepts_k_att_override():
    """k_att 只作用于引力项，不应透传给 obstacle_repulsion 触发 TypeError。"""
    fx, fy = field.total_force((0.0, 0.0), (100.0, 0.0), [], k_att=2.0)
    assert abs(fx - 2.0) < 1e-9
    assert abs(fy) < 1e-9
    ux, uy = field.desired_heading((0.0, 0.0), (100.0, 0.0), [], k_att=2.0)
    assert abs(ux - 1.0) < 1e-9 and abs(uy) < 1e-9


def test_trail_and_history_are_bounded():
    """长时间运行时轨迹与历史只保留最近一段，不会无限增长。"""
    robot = Robot(start=(0.0, 0.0), goal=(10000.0, 0.0), obstacles=[])
    for _ in range(config.TRAIL_MAX + 20):
        robot.step()
    assert len(robot.trail) <= config.TRAIL_MAX
    assert len(robot.history) <= config.HISTORY_MAX


def test_max_steps_marks_timeout():
    """一直无法抵达时，到达最大步数后停止推进并标记 timeout。"""
    robot = Robot(start=(0.0, 0.0), goal=(10000.0, 0.0), obstacles=[])
    for _ in range(config.MAX_STEPS + 1):
        robot.step()
    assert robot.done is True
    assert robot.status == "timeout"

