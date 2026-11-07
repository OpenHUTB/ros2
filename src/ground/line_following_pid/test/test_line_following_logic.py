import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from line_following_pid.pid import PID
from line_following_pid.track import build_track, nearest_point, cross_track_error
from line_following_pid.car import Car
from line_following_pid import config


def test_pid_proportional():
    p = PID(0.5, 0.0, 0.0)
    assert abs(p.step(4.0) - 2.0) < 1e-9


def test_pid_output_limit():
    p = PID(10.0, 0.0, 0.0, max_total=3.0)
    assert p.step(100.0) == 3.0


def test_pid_reset_clears_integral():
    p = PID(0.0, 1.0, 0.0)
    p.step(2.0)
    p.reset()
    assert p.integral == 0.0


def test_track_has_points():
    pts = build_track()
    assert len(pts) > 20
    assert pts[0][0] == config.TRACK_X0


def test_nearest_point_on_track_zero_error():
    pts = build_track()
    p = pts[10]
    idx = nearest_point(pts, p)
    err = cross_track_error(pts, p, 0.0, idx)
    assert abs(err) < 1e-6


def test_car_stays_near_track():
    """跑完整条赛道，横向误差应保持在合理范围内。"""
    track = build_track()
    car = Car(track)
    max_err = 0.0
    for _ in range(600):
        if car.finished():
            break
        car.step()
        max_err = max(max_err, abs(car.err))
    assert max_err < 30.0, max_err
