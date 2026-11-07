import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bug2_wall_following.bug2 import Bug2, State
from bug2_wall_following import config
from bug2_wall_following.geometry import point_segment_dist, segments_cross, circle_obstacle_nearest


def test_starts_goal_seeking():
    sim = Bug2()
    assert sim.state == State.GOAL_SEEKING


def test_moves_toward_goal_when_clear():
    # 把障碍挪开，确保直线无遮挡
    sim = Bug2(obstacle=(0.0, 0.0, 1.0))
    p0 = tuple(sim.pos)
    for _ in range(20):
        sim.step()
    assert math.hypot(sim.pos[0] - p0[0], sim.pos[1] - p0[1]) > 1.0


def test_enters_wall_following_when_blocked():
    sim = Bug2()  # 默认障碍在直线路径上
    for _ in range(800):
        sim.step()
        if sim.state == State.WALL_FOLLOWING:
            break
    assert sim.state == State.WALL_FOLLOWING
    assert sim.hit_point is not None


def test_eventually_reaches_goal():
    sim = Bug2()
    for _ in range(4000):
        sim.step()
        if sim.state == State.REACHED:
            break
    assert sim.state == State.REACHED


def test_geometry_line_dist():
    d, _ = point_segment_dist((400, 300), config.START, config.GOAL)
    assert d > 0  # 障碍中心不在直线上


def test_geometry_obstacle_nearest():
    surface, gap = circle_obstacle_nearest((400, 200), (400, 300, 70))
    assert abs(gap - 30.0) < 1.0
