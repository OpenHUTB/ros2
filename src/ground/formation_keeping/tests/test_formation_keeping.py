import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


import math


from formation_keeping import config, formation
from formation_keeping.team import Team




def test_rotate_quarter_turn():
    x, y = formation.rotate((10.0, 0.0), math.pi / 2.0)
    assert abs(x) < 1e-9
    assert abs(y - 10.0) < 1e-9




def test_desired_positions_heading_zero():
    des = formation.desired_positions((0.0, 0.0), 0.0)
    assert len(des) == 3
    assert abs(des[0][0]) < 1e-9 and abs(des[0][1]) < 1e-9
    assert abs(des[1][0] - config.FOLLOWER_OFFSETS[0][0]) < 1e-9
    assert abs(des[1][1] - config.FOLLOWER_OFFSETS[0][1]) < 1e-9




def test_desired_positions_rotate_with_heading():
    des = formation.desired_positions((0.0, 0.0), math.pi / 2.0)
    off = config.FOLLOWER_OFFSETS[0]
    rx, ry = formation.rotate(off, math.pi / 2.0)
    assert abs(des[1][0] - rx) < 1e-9
    assert abs(des[1][1] - ry) < 1e-9




def test_leader_stays_on_circle():
    for step in (0, 37, 128, 401):
        pos, heading = formation.leader_state(step)
        r = math.hypot(pos[0] - config.CENTER[0], pos[1] - config.CENTER[1])
        assert abs(r - config.RADIUS) < 1e-6
        a = config.PHASE0 + config.OMEGA * step
        ang = math.atan2(pos[1] - config.CENTER[1], pos[0] - config.CENTER[0])
        diff = ang - a
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        assert abs(diff) < 1e-6




def test_formation_error_is_zero_for_exact_positions():
    pos = [(0.0, 0.0), (10.0, 0.0)]
    assert formation.formation_error(pos, pos) == 0.0




def test_team_converges_to_formation():
    team = Team()
    initial_error = team.error()
    assert initial_error > 300.0
    steps = 0
    while steps < 400:
        team.step()
        steps += 1
    # 收敛过程：100 步内误差应降到初始一半以下
    err_at_100 = team.errors[99]
    assert err_at_100 < initial_error * 0.5
    assert team.error() <= config.TOL
    assert team.settled() is True




def test_formation_held_after_settling():
    team = Team()
    for _ in range(1000):
        team.step()
    assert team.error() <= config.TOL
    assert max(team.errors[config.SETTLE_STEPS:]) <= config.TOL
